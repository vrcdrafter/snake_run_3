extends Node3D
## Head-directed locomotion with active thrust and a travelling body wave.

@export var target_path: NodePath = ^"../Target"
@export_range(0.1, 8.0, 0.1) var head_speed := 3.6
@export_range(0.1, 20.0, 0.1) var head_steering_gain := 3.5
@export_range(0.1, 30.0, 0.1) var head_acceleration_limit := 18.0
@export_range(0.1, 20.0, 0.1) var braking_acceleration := 6.0
@export_range(0.0, 5.0, 0.05) var stop_radius := 0.75
@export_range(0.1, 8.0, 0.1) var stop_fade_distance := 2.0

@export_range(0.0, 25.0, 0.1) var segment_acceleration := 12.0
@export_range(0.1, 3.0, 0.05) var tail_drive_multiplier := 1.15
@export_range(0.0, 1.0, 0.01) var thrust_wave_amount := 0.15

@export_range(0.1, 10.0, 0.1) var wave_frequency := 3.3
@export_range(0.1, 3.0, 0.05) var wave_number := 0.9
@export_range(0.0, 2.0, 0.05) var wave_amplitude := 0.65
@export_range(0.0, 10.0, 0.1) var alignment_strength := 3.5
@export_range(0.0, 5.0, 0.05) var alignment_damping := 0.8
@export_range(0.05, 10.0, 0.05) var alignment_torque_limit := 2.5
@export_range(0.0, 10.0, 0.1) var roll_strength := 3.0
@export_range(0.0, 5.0, 0.05) var roll_damping := 1.5

@export_range(0.0, 5.0, 0.05) var body_linear_damp := 1.0
@export_range(0.0, 5.0, 0.05) var body_angular_damp := 0.9

@onready var simulator: PhysicalBoneSimulator3D = $snake_export/Skeleton3D/PhysicalBoneSimulator3D

var _target: Node3D
var _bones: Array[PhysicalBone3D] = []
var _collision_shapes: Array[CollisionShape3D] = []
var _bone_local_test := false
var _elapsed := 0.0
var _last_heading := Vector3.FORWARD


func _ready() -> void:
	_target = get_node_or_null(target_path) as Node3D
	if _target == null:
		push_warning("SnakeSlitherController: target_path does not point to a Node3D: %s" % target_path)

	for child in simulator.get_children():
		if child is PhysicalBone3D:
			var bone := child as PhysicalBone3D
			if bone.has_method("is_bone_local_drive_test") and bool(bone.call("is_bone_local_drive_test")):
				_bone_local_test = true
			bone.can_sleep = false
			bone.linear_damp = body_linear_damp
			bone.angular_damp = body_angular_damp
			_bones.append(bone)

	_bones.sort_custom(func(a, b):
		return a.get_bone_id() < b.get_bone_id()
	)
	for bone in _bones:
		_collision_shapes.append(bone.get_node_or_null("CollisionShape3D") as CollisionShape3D)

	if _bones.is_empty():
		push_error("SnakeSlitherController: no PhysicalBone3D children found")
		set_physics_process(false)
		return

	simulator.physical_bones_start_simulation()
	if _bone_local_test:
		set_physics_process(false)


func _physics_process(delta: float) -> void:
	if _target == null or _bones.is_empty():
		return

	var head := _bones[0]
	var to_target := _target.global_position - head.global_position
	to_target.y = 0.0
	var target_distance := to_target.length()
	if target_distance > 0.0001:
		_last_heading = to_target / target_distance

	_elapsed += delta
	var distance_to_move := maxf(target_distance - stop_radius, 0.0)
	var drive_amount := clampf(distance_to_move / stop_fade_distance, 0.0, 1.0)
	var desired_speed := minf(head_speed, sqrt(2.0 * braking_acceleration * distance_to_move))
	var desired_velocity := _last_heading * desired_speed
	var head_velocity := head.linear_velocity
	head_velocity.y = 0.0
	var steering_acceleration := (desired_velocity - head_velocity) * head_steering_gain
	steering_acceleration = steering_acceleration.limit_length(head_acceleration_limit)
	PhysicsServer3D.body_apply_central_force(head.get_rid(), steering_acceleration * head.mass)

	var distance_along_body := 0.0
	for index in _bones.size():
		var bone := _bones[index]
		if index > 0:
			distance_along_body += bone.global_position.distance_to(_bones[index - 1].global_position)

		var tangent := _segment_tangent(index)
		var phase := _elapsed * wave_frequency - distance_along_body * wave_number
		var desired_axis := tangent
		if index > 0:
			var side := Vector3.UP.cross(tangent)
			if side.length_squared() > 0.0001:
				side = side.normalized()
				var wave_slope := wave_amplitude * wave_number * cos(phase) * drive_amount
				desired_axis = (tangent + side * wave_slope).normalized()

		_apply_axis_alignment(index, bone, desired_axis)
		_apply_segment_drive(index, bone, tangent, phase, drive_amount)


func _segment_tangent(index: int) -> Vector3:
	if index == 0:
		return _last_heading

	var toward_head := _bones[index - 1].global_position - _bones[index].global_position
	toward_head.y = 0.0
	if toward_head.length_squared() < 0.0001:
		return _last_heading
	return toward_head.normalized()


func _apply_segment_drive(index: int, bone: PhysicalBone3D, forward_guide: Vector3, phase: float, drive_amount: float) -> void:
	if drive_amount <= 0.001:
		return

	var axis := bone.global_transform.basis.y
	var collision_shape := _collision_shapes[index]
	if collision_shape != null:
		axis = collision_shape.global_transform.basis.y
	if axis.length_squared() < 0.0001:
		return
	axis = axis.normalized()
	if axis.dot(forward_guide) < 0.0:
		axis = -axis

	var tail_fraction := float(index) / float(maxi(_bones.size() - 1, 1))
	var tail_scale := lerpf(1.0, tail_drive_multiplier, tail_fraction)
	var wave_scale := 1.0 + thrust_wave_amount * sin(phase)
	var force := axis * bone.mass * segment_acceleration * tail_scale * wave_scale * drive_amount
	PhysicsServer3D.body_apply_central_force(bone.get_rid(), force)


func _apply_axis_alignment(index: int, bone: PhysicalBone3D, desired_axis: Vector3) -> void:
	var current_axis := bone.global_transform.basis.y
	var collision_shape := _collision_shapes[index]
	if collision_shape != null:
		current_axis = collision_shape.global_transform.basis.y
	if current_axis.length_squared() < 0.0001:
		return
	current_axis = current_axis.normalized()
	# Remove angular velocity around the segment's long axis: this is roll.
	var roll_spin := bone.angular_velocity.dot(current_axis)
	bone.angular_velocity -= current_axis * roll_spin
	if current_axis.dot(desired_axis) < 0.0:
		desired_axis = -desired_axis

	var perpendicular_spin := bone.angular_velocity - current_axis * bone.angular_velocity.dot(current_axis)
	var torque := current_axis.cross(desired_axis) * alignment_strength - perpendicular_spin * alignment_damping
	var dorsal_axis := bone.global_transform.basis.x
	if collision_shape != null:
		dorsal_axis = collision_shape.global_transform.basis.z
	dorsal_axis -= current_axis * dorsal_axis.dot(current_axis)
	var upright_axis := Vector3.UP - current_axis * Vector3.UP.dot(current_axis)
	if dorsal_axis.length_squared() > 0.0001 and upright_axis.length_squared() > 0.0001:
		dorsal_axis = dorsal_axis.normalized()
		upright_axis = upright_axis.normalized()
		var roll_error := atan2(current_axis.dot(dorsal_axis.cross(upright_axis)), clampf(dorsal_axis.dot(upright_axis), -1.0, 1.0))
		var roll_speed := bone.angular_velocity.dot(current_axis)
		torque += current_axis * (roll_error * roll_strength - roll_speed * roll_damping)
	torque = torque.limit_length(alignment_torque_limit)
	PhysicsServer3D.body_apply_torque(bone.get_rid(), torque)
