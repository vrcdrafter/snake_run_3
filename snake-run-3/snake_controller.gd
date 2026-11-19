extends Node3D
## Drives the imported physical-bone chain toward a scene target.
## Steering is force based: the head pulls toward the goal while a travelling
## lateral wave bends the remaining bodies through small alignment torques.

@export var target_path: NodePath = ^"../Target"
@export_range(0.1, 8.0, 0.1) var head_speed := 2.4
@export_range(0.1, 40.0, 0.1) var head_drive := 12.0
@export_range(1.0, 100.0, 0.5) var head_force_limit := 30.0
@export_range(0.0, 5.0, 0.01) var alignment_strength := 0.22
@export_range(0.0, 2.0, 0.01) var alignment_damping := 0.15
@export_range(0.05, 5.0, 0.01) var alignment_torque_limit := 0.25
@export_range(0.0, 10.0, 0.1) var lateral_force := 1.0
@export_range(0.1, 10.0, 0.1) var wave_frequency := 2.2
@export_range(0.1, 3.0, 0.05) var wave_number := 0.9
@export_range(0.0, 2.0, 0.05) var wave_amplitude := 0.65

@onready var simulator: PhysicalBoneSimulator3D = $snake_export/Skeleton3D/PhysicalBoneSimulator3D

var _target: Node3D
var _bones: Array[PhysicalBone3D] = []
var _elapsed := 0.0
var _last_heading := Vector3.FORWARD


func _ready() -> void:
	_target = get_node_or_null(target_path) as Node3D
	if _target == null:
		push_warning("SnakeController: target_path does not point to a Node3D: %s" % target_path)

	for child in simulator.get_children():
		if child is PhysicalBone3D:
			var bone := child as PhysicalBone3D
			bone.can_sleep = false
			_bones.append(bone)

	_bones.sort_custom(func(a, b):
		return a.get_bone_id() < b.get_bone_id()
	)

	if _bones.is_empty():
		push_error("SnakeController: no PhysicalBone3D children found")
		set_physics_process(false)
		return

	simulator.physical_bones_start_simulation()


func _physics_process(delta: float) -> void:
	if _target == null or _bones.is_empty():
		return

	_elapsed += delta
	var head := _bones[0]
	var to_target := _target.global_position - head.global_position
	to_target.y = 0.0
	var target_distance := to_target.length()
	if target_distance > 0.0001:
		_last_heading = to_target.normalized()

	# A velocity spring gives the head a controllable top speed and brakes it
	# as it reaches the target, without teleporting the physical body.
	var desired_speed := minf(head_speed, target_distance * 1.5)
	var desired_velocity := _last_heading * desired_speed
	var current_velocity := head.linear_velocity
	current_velocity.y = 0.0
	var drive_force := (desired_velocity - current_velocity) * head_drive * head.mass
	if drive_force.length() > head_force_limit:
		drive_force = drive_force.normalized() * head_force_limit
	PhysicsServer3D.body_apply_central_force(head.get_rid(), drive_force)

	for index in _bones.size():
		var bone := _bones[index]
		var tangent := _segment_tangent(index, _last_heading)
		var side := Vector3.UP.cross(tangent).normalized()
		var phase := _elapsed * wave_frequency - float(index) * wave_number * 0.5
		var wave_slope := wave_amplitude * wave_number * cos(phase)
		var desired_axis := (tangent + side * wave_slope).normalized()

		_apply_axis_alignment(bone, desired_axis)

		# Side forces are small and phase-shift down the chain, so the body
		# develops a travelling bend instead of moving as a rigid bar.
		if index > 0:
			var tail_weight := 0.35 + 0.65 * float(index) / float(_bones.size() - 1)
			var lateral := side * sin(phase) * lateral_force * tail_weight
			PhysicsServer3D.body_apply_central_force(bone.get_rid(), lateral)


func _segment_tangent(index: int, fallback: Vector3) -> Vector3:
	if index == 0:
		return fallback

	var toward_head := _bones[index - 1].global_position - _bones[index].global_position
	toward_head.y = 0.0
	if toward_head.length_squared() < 0.0001:
		return fallback
	return toward_head.normalized()


func _apply_axis_alignment(bone: PhysicalBone3D, desired_axis: Vector3) -> void:
	var collision_shape := bone.get_node_or_null("CollisionShape3D") as CollisionShape3D
	var current_axis := bone.global_transform.basis.y
	if collision_shape != null:
		# CapsuleShape3D is long along its local Y axis. Use the transformed
		# shape axis so the controller follows this imported rig's orientation.
		current_axis = collision_shape.global_transform.basis.y
	if current_axis.length_squared() < 0.0001:
		return
	current_axis = current_axis.normalized()

	# A capsule has no meaningful forward/backward direction, so align its
	# nearest axis orientation and avoid a needless half-turn.
	if current_axis.dot(desired_axis) < 0.0:
		desired_axis = -desired_axis

	var perpendicular_spin := bone.angular_velocity - current_axis * bone.angular_velocity.dot(current_axis)
	var torque := current_axis.cross(desired_axis) * alignment_strength - perpendicular_spin * alignment_damping
	if torque.length() > alignment_torque_limit:
		torque = torque.normalized() * alignment_torque_limit
	PhysicsServer3D.body_apply_torque(bone.get_rid(), torque)
