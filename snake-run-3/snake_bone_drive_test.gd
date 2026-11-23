extends PhysicalBone3D
## Test controller attached directly to each physical bone.

const SELF_BODY_RAY_LENGTH := 0.9
const SELF_BODY_RAY_SIDE_SPREAD := 0.16
const SELF_BODY_NEAR_HEAD_DISTANCE := 0.85
const SELF_BODY_PATH_LENGTH := 1.35
const SELF_BODY_PATH_RADIUS := 0.55
const SELF_BODY_LIFT_HEIGHT := 0.7
const SELF_BODY_LIFT_HOLD_TIME := 0.7
const SELF_BODY_LIFT_RISE_SPEED := 1.5
const SELF_BODY_LIFT_DESCENT_SPEED := 0.8

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
var _nearby_bone_rids: Array[RID] = []
var _other_bones: Array[PhysicalBone3D] = []
var _elapsed := 0.0
var _last_heading := Vector3.FORWARD
var _head_base_height := 0.0
var _climb_hold_remaining := 0.0
var _climb_offset := 0.0


func is_bone_local_drive_test() -> bool:
	return _head != null and _target != null


func is_self_climbing() -> bool:
	return _climb_offset > 0.01


func _ready() -> void:
	_simulator = get_parent() as PhysicalBoneSimulator3D
	if _simulator == null:
		push_error("SnakeBoneDriveTest: expected parent PhysicalBoneSimulator3D")
		set_physics_process(false)
		return

	_owner_root = _simulator.get_parent().get_parent().get_parent() as Node3D
	_target = _owner_root.get_node_or_null(target_path) as Node3D
	if _target == null:
		var scene_root := get_tree().current_scene
		if scene_root != null:
			_target = scene_root.find_child("Target", true, false) as Node3D
	_collision_shape = get_node_or_null("CollisionShape3D") as CollisionShape3D
	can_sleep = false

	var physical_bones: Array[PhysicalBone3D] = []
	for child in _simulator.get_children():
		if child is PhysicalBone3D:
			physical_bones.append(child as PhysicalBone3D)

	_bone_index = physical_bones.find(self)
	_bone_count = physical_bones.size()
	if not physical_bones.is_empty():
		_head = physical_bones[0]
	if _bone_index > 0:
		_previous_bone = physical_bones[_bone_index - 1]
	for index in physical_bones.size():
		var other_bone := physical_bones[index]
		if abs(index - _bone_index) <= 2:
			_nearby_bone_rids.append(other_bone.get_rid())
		else:
			_other_bones.append(other_bone)

	if _bone_index == 0:
		_head_base_height = global_position.y

	if _head == null:
		push_error("SnakeBoneDriveTest: physical head bone was not found")
		set_physics_process(false)
		return
	if _bone_index < 0:
		push_error("SnakeBoneDriveTest: physical bone is missing from simulator child list")
		set_physics_process(false)
		return
	if _target == null:
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

	if _bone_index == 0:
		_update_self_climb(delta)

	var distance_to_move := maxf(target_distance - stop_radius, 0.0)
	var drive_amount := clampf(distance_to_move / stop_fade_distance, 0.0, 1.0)
	if _bone_index == 0:
		_apply_head_navigation(distance_to_move)

	var forward_guide := _last_heading
	if _bone_index > 0 and _previous_bone != null:
		forward_guide = _previous_bone.global_position - global_position
		if not bool(_head.call("is_self_climbing")):
			forward_guide.y = 0.0
		if forward_guide.length_squared() > 0.0001:
			forward_guide = forward_guide.normalized()
		else:
			forward_guide = _last_heading

	var phase := _elapsed * wave_frequency - float(_bone_index) * segment_spacing * wave_number
	var desired_axis := forward_guide
	if _bone_index == 0 and _climb_offset > 0.01:
		var climb_fraction := clampf(_climb_offset / SELF_BODY_LIFT_HEIGHT, 0.0, 1.0)
		desired_axis = (forward_guide + Vector3.UP * 0.75 * climb_fraction).normalized()
	elif _bone_index > 0:
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
	var vertical_control_active := _climb_offset > 0.001 or global_position.y > _head_base_height + 0.03
	if vertical_control_active:
		var desired_height := _head_base_height + _climb_offset
		var height_error := desired_height - global_position.y
		desired_velocity.y = clampf(height_error * 3.0, -SELF_BODY_LIFT_DESCENT_SPEED, SELF_BODY_LIFT_RISE_SPEED)
	else:
		head_velocity.y = 0.0
	var steering_acceleration := (desired_velocity - head_velocity) * head_steering_gain
	steering_acceleration = steering_acceleration.limit_length(head_acceleration_limit)
	if vertical_control_active:
		steering_acceleration.y += float(ProjectSettings.get_setting("physics/3d/default_gravity"))
	PhysicsServer3D.body_apply_central_force(get_rid(), steering_acceleration * mass)


func _update_self_climb(delta: float) -> void:
	if _is_other_body_ahead():
		_climb_hold_remaining = SELF_BODY_LIFT_HOLD_TIME
	else:
		_climb_hold_remaining = maxf(_climb_hold_remaining - delta, 0.0)

	var target_offset := SELF_BODY_LIFT_HEIGHT if _climb_hold_remaining > 0.0 else 0.0
	var lift_speed := SELF_BODY_LIFT_RISE_SPEED if target_offset > _climb_offset else SELF_BODY_LIFT_DESCENT_SPEED
	_climb_offset = move_toward(_climb_offset, target_offset, lift_speed * delta)


func _is_other_body_ahead() -> bool:
	for other_bone in _other_bones:
		if not is_instance_valid(other_bone):
			continue
		var offset := other_bone.global_position - global_position
		if offset.length_squared() <= SELF_BODY_NEAR_HEAD_DISTANCE * SELF_BODY_NEAR_HEAD_DISTANCE:
			return true

		var flat_offset := Vector3(offset.x, 0.0, offset.z)
		var distance_ahead := flat_offset.dot(_last_heading)
		if distance_ahead < 0.0 or distance_ahead > SELF_BODY_PATH_LENGTH:
			continue
		var lateral_offset := flat_offset - _last_heading * distance_ahead
		lateral_offset.y = offset.y
		if lateral_offset.length_squared() <= SELF_BODY_PATH_RADIUS * SELF_BODY_PATH_RADIUS:
			return true

	var side := Vector3.UP.cross(_last_heading)
	if side.length_squared() < 0.0001:
		return false
	side = side.normalized()
	var space_state := get_world_3d().direct_space_state
	for side_offset in [-SELF_BODY_RAY_SIDE_SPREAD, 0.0, SELF_BODY_RAY_SIDE_SPREAD]:
		var origin :Vector3 = global_position + Vector3.UP * 0.08 + side * side_offset + _last_heading * 0.12
		var query := PhysicsRayQueryParameters3D.create(origin, origin + _last_heading * SELF_BODY_RAY_LENGTH)
		query.exclude = _nearby_bone_rids
		query.hit_from_inside = true
		var hit := space_state.intersect_ray(query)
		if hit.is_empty():
			continue
		var collider := hit.get("collider") as PhysicalBone3D
		if collider != null and collider.get_parent() == _simulator:
			return true
	return false


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
