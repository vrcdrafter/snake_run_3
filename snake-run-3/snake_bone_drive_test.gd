extends PhysicalBone3D
## Test controller attached directly to each physical bone.

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
@export_range(0.1, 2.0, 0.05) var segment_spacing := 0.5
@export_range(0.0, 2.0, 0.05) var wave_amplitude := 0.65

@export_range(0.0, 10.0, 0.1) var alignment_strength := 3.5
@export_range(0.0, 5.0, 0.05) var alignment_damping := 0.8
@export_range(0.05, 10.0, 0.05) var alignment_torque_limit := 2.5
@export_range(0.0, 10.0, 0.1) var roll_strength := 3.0
@export_range(0.0, 5.0, 0.05) var roll_damping := 1.5

var _simulator: PhysicalBoneSimulator3D
var _owner_root: Node3D
var _target: Node3D
var _head: PhysicalBone3D
var _previous_bone: PhysicalBone3D
var _collision_shape: CollisionShape3D
var _bone_index := 0
var _bone_count := 0
var _elapsed := 0.0
var _last_heading := Vector3.FORWARD


func is_bone_local_drive_test() -> bool:
	return _head != null and _target != null


func _ready() -> void:
	_simulator = get_parent() as PhysicalBoneSimulator3D
	if _simulator == null:
		push_error("SnakeBoneDriveTest: expected parent PhysicalBoneSimulator3D")
		set_physics_process(false)
		return

	_owner_root = _simulator.get_parent().get_parent().get_parent() as Node3D
	_bone_index = get_bone_id()
	_target = _owner_root.get_node_or_null(target_path) as Node3D
	if _target == null:
		var scene_root := get_tree().current_scene
		if scene_root != null:
			_target = scene_root.find_child("Target", true, false) as Node3D
	_collision_shape = get_node_or_null("CollisionShape3D") as CollisionShape3D
	can_sleep = false

	for child in _simulator.get_children():
		if child is PhysicalBone3D:
			var other_bone := child as PhysicalBone3D
			if other_bone.get_bone_id() == 0:
				_head = other_bone
			if other_bone.get_bone_id() == _bone_index - 1:
				_previous_bone = other_bone
			_bone_count += 1

	if _head == null:
		push_error("SnakeBoneDriveTest: physical head bone was not found")
		set_physics_process(false)
	elif _target == null:
		if _bone_index == 0:
			push_warning("SnakeBoneDriveTest: target was not found; root controller will take over")
		set_physics_process(false)


func _physics_process(delta: float) -> void:
	if _head == null:
		return

	_elapsed += delta
	var target_distance := 0.0
	if _target != null:
		var to_target := _target.global_position - _head.global_position
		to_target.y = 0.0
		target_distance = to_target.length()
		if target_distance > 0.0001:
			_last_heading = to_target / target_distance

	var distance_to_move := maxf(target_distance - stop_radius, 0.0)
	var drive_amount := clampf(distance_to_move / stop_fade_distance, 0.0, 1.0)
	if _bone_index == 0:
		_apply_head_navigation(distance_to_move)

	var forward_guide := _last_heading
	if _bone_index > 0 and _previous_bone != null:
		forward_guide = _previous_bone.global_position - global_position
		forward_guide.y = 0.0
		if forward_guide.length_squared() > 0.0001:
			forward_guide = forward_guide.normalized()
		else:
			forward_guide = _last_heading

	var phase := _elapsed * wave_frequency - float(_bone_index) * segment_spacing * wave_number
	var desired_axis := forward_guide
	if _bone_index > 0:
		var side := Vector3.UP.cross(forward_guide)
		if side.length_squared() > 0.0001:
			side = side.normalized()
			var wave_slope := wave_amplitude * wave_number * cos(phase) * drive_amount
			desired_axis = (forward_guide + side * wave_slope).normalized()

	_apply_axis_alignment(desired_axis)
	_apply_segment_drive(forward_guide, phase, drive_amount)


func _apply_head_navigation(distance_to_move: float) -> void:
	var desired_speed := minf(head_speed, sqrt(2.0 * braking_acceleration * distance_to_move))
	var desired_velocity := _last_heading * desired_speed
	var head_velocity := linear_velocity
	head_velocity.y = 0.0
	var steering_acceleration := (desired_velocity - head_velocity) * head_steering_gain
	steering_acceleration = steering_acceleration.limit_length(head_acceleration_limit)
	PhysicsServer3D.body_apply_central_force(get_rid(), steering_acceleration * mass)


func _apply_segment_drive(forward_guide: Vector3, phase: float, drive_amount: float) -> void:
	if drive_amount <= 0.001:
		return

	var axis := global_transform.basis.y
	if _collision_shape != null:
		axis = _collision_shape.global_transform.basis.y
	if axis.length_squared() < 0.0001:
		return
	axis = axis.normalized()
	if axis.dot(forward_guide) < 0.0:
		axis = -axis

	var tail_fraction := float(_bone_index) / float(maxi(_bone_count - 1, 1))
	var tail_scale := lerpf(1.0, tail_drive_multiplier, tail_fraction)
	var wave_scale := 1.0 + thrust_wave_amount * sin(phase)
	var force := axis * mass * segment_acceleration * tail_scale * wave_scale * drive_amount
	PhysicsServer3D.body_apply_central_force(get_rid(), force)


func _apply_axis_alignment(desired_axis: Vector3) -> void:
	var current_axis := global_transform.basis.y
	if _collision_shape != null:
		current_axis = _collision_shape.global_transform.basis.y
	if current_axis.length_squared() < 0.0001:
		return
	current_axis = current_axis.normalized()
	# Remove angular velocity around the segment's long axis: this is roll.
	var roll_spin := angular_velocity.dot(current_axis)
	angular_velocity -= current_axis * roll_spin
	if current_axis.dot(desired_axis) < 0.0:
		desired_axis = -desired_axis

	var perpendicular_spin := angular_velocity - current_axis * angular_velocity.dot(current_axis)
	var torque := current_axis.cross(desired_axis) * alignment_strength - perpendicular_spin * alignment_damping
	var dorsal_axis := global_transform.basis.x
	if _collision_shape != null:
		dorsal_axis = _collision_shape.global_transform.basis.z
	dorsal_axis -= current_axis * dorsal_axis.dot(current_axis)
	var upright_axis := Vector3.UP - current_axis * Vector3.UP.dot(current_axis)
	if dorsal_axis.length_squared() > 0.0001 and upright_axis.length_squared() > 0.0001:
		dorsal_axis = dorsal_axis.normalized()
		upright_axis = upright_axis.normalized()
		var roll_error := atan2(current_axis.dot(dorsal_axis.cross(upright_axis)), clampf(dorsal_axis.dot(upright_axis), -1.0, 1.0))
		var roll_speed := angular_velocity.dot(current_axis)
		torque += current_axis * (roll_error * roll_strength - roll_speed * roll_damping)
	torque = torque.limit_length(alignment_torque_limit)
	PhysicsServer3D.body_apply_torque(get_rid(), torque)
