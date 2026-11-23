extends Marker3D
## Moves the visible target across the arena with WASD.

@export_range(0.1, 20.0, 0.1) var move_speed := 5.0
@export var arena_x_limit := 13.0
@export var arena_z_limit := 19.0


func _process(delta: float) -> void:
	var direction := Vector2.ZERO
	if Input.is_key_pressed(KEY_A):
		direction.x -= 1.0
	if Input.is_key_pressed(KEY_D):
		direction.x += 1.0
	if Input.is_key_pressed(KEY_W):
		direction.y -= 1.0
	if Input.is_key_pressed(KEY_S):
		direction.y += 1.0

	if direction == Vector2.ZERO:
		return

	direction = direction.normalized()
	var next_position := global_position
	next_position.x = clampf(next_position.x + direction.x * move_speed * delta, -arena_x_limit, arena_x_limit)
	next_position.z = clampf(next_position.z + direction.y * move_speed * delta, -arena_z_limit, arena_z_limit)
	global_position = next_position
