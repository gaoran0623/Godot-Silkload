class_name ClothsimModifier
extends SkeletonModifier3D


var isReady: bool = false
var PM_list = []
var link_list = []
var spawn_offset = Vector3(0, 0,0)
var PM_spacing = 0.2 # Length of links between nodes
@onready var skeleton: Skeleton3D= get_skeleton()

##For debugging
@export var nodeDisabled: bool =false

##For debugging
@export var showColliderIndicator: bool =false

##decide if the verlet particle should be able to collide at the root.Normally the root will NOT be colliding, but who knows
@export var collideAtRoot: bool =false

##Sometimes you don't want the whole bonechain to be simulated like cloth, in case that you need part of the bone chain to act
@export var trim_bonechain_by: int =0

##Note this will be the biggest collider's radius, but you can use the curve to derate along the bone chain
@export_custom(PROPERTY_HINT_NONE, "suffix:m") var collider_shape_radius_base :float =0.04

##Along the bone chain that existing available colliders, set the derating ratio of the colliders size. the trimmed bones will not be counted
@export var collider_size_curve :Curve = Curve.new()

##Simulate a cylinder-like cloth such as skirt: true, or a piece of cloth: false
@export var closedBoneLoop: bool =false

@export_flags_3d_physics  var collider_collision_layer: int = 0
@export_flags_3d_physics  var collider_collision_mask: int = 4




func _process_modification() -> void:
		#skeleton = get_skeleton()
		if !skeleton:
			return # Never happen, but for the safety.
		_bone_points_to_target()

func _bone_points_to_target():
	for i in boneIndexMatrix.size():
		for j in boneIndexMatrix[i].size()-trim_bonechain_by:
			var _bone_idx: int = boneIndexMatrix[i][j]
			_process_lookAt(_bone_idx, PM_list[i][j+1]) 

###############################################

class nonCollisionPoint:
	extends Node3D
	var velocity = Vector3(0,0,0)
	var inertiaForce  :Vector3 = Vector3(0,0,0)
	var massFactor = 0.0
	var acceleration = Vector3(0,-massFactor,0) # Gravity or other external forces. 
	var entered_body = false;
	var is_pin = false
	var collision_factor = 1.0 # Read docs for what this does
	var dampen_factor :float= .98#0.98
	@onready var last_global_position = global_position
	@onready var last_global_trans_basis_inverse= global_transform.basis.inverse()
	@onready var last_position = position
	func _ready():
		pass
	func show_collider_indicator():
		var indicator = MeshInstance3D.new()
		var shape=SphereMesh.new()
		var shapeRadius:float=0.01
		shape.radius=shapeRadius
		shape.height=shape.radius*2
		shape.radial_segments=6
		shape.rings=3
		indicator.mesh = shape
		self.add_child(indicator)	
	func do_verlet(delta):
		var accelerationFactor = 1  
		var inertiaForceFactor = 1
		velocity = position - last_position
		inertiaForce=global_transform.basis.inverse()*(global_position-last_global_position)
		last_position = position
		last_global_position = global_position
		last_global_trans_basis_inverse=global_transform.basis.inverse()
		velocity *= dampen_factor # damping
		position = position + (velocity * delta * 60) + (acceleration * delta * accelerationFactor) + (inertiaForce * delta * 1)


###############################################

class collisionPoint:
	extends RigidBody3D
	var velocity :Vector3= Vector3(0,0,0)
	var inertiaForce  :Vector3 = Vector3(0,0,0)
	var colSize :float = 0.038
	var massFactor :float= 0.01
	var grvty = Vector3(0,-9.8,0) # Gravity or other external forces,NOT using native mass of RigidBody3D
	var entered_body :bool = false;
	var is_pin :bool= false
	var collision_factor :float= 1 # Read docs for what this does
	var dampen_factor :float= .96#0.98
	@onready var last_global_position = global_position
	@onready var last_global_trans_basis_inverse= global_transform.basis.inverse()
	@onready var last_position = position
	func _ready():
		mass=0.01 #the original mass is not usefull
		set_gravity_scale(0.0) #the original gravity is not usefull, and will affect the verlet integration
		continuous_cd = true
		custom_integrator = true
		inertia = Vector3.ZERO
		lock_rotation = true
		linear_damp = 0.001
		#var phyMarteriarOverride = PhysicsMaterial.new()
		#physics_material_override = phyMarteriarOverride
		#physics_material_override.friction=0
		
	func add_collider_with_size(size:float):
		var colliderShape= CollisionShape3D.new()
		var colshape= SphereShape3D.new()
		colshape.radius=size #0.038
		colshape.margin=0.01
		colliderShape.shape=colshape
		self.add_child(colliderShape)
				
	func add_indicator_with_size(size:float):
		var indicator = MeshInstance3D.new()
		var shape=SphereMesh.new()
		shape.radius=size
		shape.height=shape.radius*2
		shape.radial_segments=6
		shape.rings=3
		indicator.mesh = shape
		self.add_child(indicator)
		
	func do_verlet(delta):
		var accelerationFactor = 1  #mainly gravity
		var inertiaForceFactor = 150
		velocity = position - last_position
		inertiaForce=global_transform.basis.inverse()*(global_position-last_global_position)
		#velocity=global_transform.basis.inverse()*(global_position-last_global_position)
		last_position = position
		last_global_position = global_position
		last_global_trans_basis_inverse=global_transform.basis.inverse()
		if (body_entered): # collision
			#accelerationFactor *= 0.5
			#inertiaForceFactor = 50
			position = last_position
			velocity *= collision_factor
		velocity *= dampen_factor # damping
		#verlet algorithm
		position = position + (velocity * delta * 60) + (massFactor* grvty * delta * accelerationFactor) - (massFactor * inertiaForce *  delta * inertiaForceFactor)
		#position = 2*position - last_position + (grvty)*delta*delta*2
		#position = get_parent().get_parent().to_local(2*global_position - last_global_position) + (grvty)*delta*delta


class link:
	extends Node3D
	var resting_distance = 1 # default value, can be overriden when instanced
	var PM_a # Point Mass
	var PM_b
	var is_PM_a_pin # A pinned node's position is not affected by the constraint
	var is_PM_b_pin
	var is_rigid = false # A rigid link has a minimum length
	var color = Color(1.0, 2.0, 1.0)
# Called when the node enters the scene tree for the first time.
	func _ready():
		if (PM_a.get("is_pin") == null):
			is_PM_a_pin = true # True by default if not otherwise specified by node
		else:
			is_PM_a_pin = PM_a.is_pin # Can set value manually by giving your node an is_pin variable
		if (PM_b.get("is_pin") == null):
			is_PM_b_pin = true
		else:
			is_PM_b_pin = PM_b.is_pin
		resting_distance = PM_a.position.distance_to(PM_b.position)
	func constrain():
		var distance = PM_a.position.distance_to(PM_b.position)
		var distance_ratio
		if distance != 0:	# Prevent division by zero, which is basically impossible but hey
			distance_ratio = (resting_distance - distance) / distance
		else:
			distance_ratio = 1
		if distance_ratio <= 0 || is_rigid: # distance > resting_distance
			# Move points proportionally to how far away they are. Hook's law
			var translate_by = (PM_a.position - PM_b.position) * 0.5 * distance_ratio
			if !is_PM_a_pin:
				PM_a.position += translate_by
			if !is_PM_b_pin:
					PM_b.position -= translate_by


#
#p[0][0] p[1][0] p[2][0] p[3][0] p[4][0]
#p[0][1] p[1][1] p[2][1] p[3][1] p[4][1]
#p[0][2] p[1][2] p[2][2] p[3][2] p[4][2]
#p[0][3] p[1][3] p[2][3] p[3][3] p[4][3]
#p[0][4] p[1][4] p[2][4] p[3][4] p[4][4]
#---------------trim--------------------
#p[0][5] p[1][5] p[2][5] p[3][5] p[4][5]

func link_PM():
	assert(PM_list.size()>0)
	for i in PM_list.size(): # Vertical
		for j in PM_list[i].size()-1:
			var new_link = link.new() 
			new_link.PM_a = PM_list[i][j]
			new_link.PM_b = PM_list[i][j+1]
			#new_link.resting_distance = PM_spacing
			link_list.append(new_link)
			add_child(new_link) # Links must be added to scene to draw their lines
			
	for i in PM_list.size()-1:
		for j in PM_list[i].size(): # Horizontal
			var new_link = link.new() 
			new_link.PM_a = PM_list[i][j]
			new_link.PM_b = PM_list[i+1][j]
			#new_link.resting_distance = PM_spacing
			link_list.append(new_link)
			add_child(new_link)
	if closedBoneLoop:
		for j in PM_list[PM_list.size()-1].size()-1: #the last column
			var new_link = link.new() 
			new_link.PM_a = PM_list[PM_list.size()-1][j]
			new_link.PM_b = PM_list[0][j]
			#new_link.resting_distance = PM_spacing
			link_list.append(new_link)
			add_child(new_link)



##If the skeleton has a lot of bones, it's not convinient to select one by one, I'd rather type the names as Strings here
@export var rootBoneNameList = PackedStringArray()
var boneIndexMatrix=[] #storing all clothsim bone indexs
var PMpositionMatrix=[]

func _initialize_bone_chain(skeleton, bone_index):#获取bone的所有bone chain建立points的列表
	var boneChain = _get_bone_chain(skeleton, bone_index)
	boneIndexMatrix.append(boneChain)
	#print("boneIndexMatrix",boneIndexMatrix)

func _prepare_positions(): #put positions into a matrix/array
	for boneChain in boneIndexMatrix:
		var positionList=[]
		for i in boneChain.size()-trim_bonechain_by:
			print("i",i)
			print("boneChain.size()-1 ",boneChain.size()-1)
			print("boneChain.size()-trim_bonechain_by ",boneChain.size()-trim_bonechain_by)
			var _position=skeleton.get_bone_global_pose(boneChain[i]).origin
			positionList.append(_position)
			if i==boneChain.size()-trim_bonechain_by-1 && i!=boneChain.size()-1: # if bone chain trimmed, use next bone origin as position
				_position=skeleton.get_bone_global_pose(boneChain[i+1]).origin
				positionList.append(_position)
				print("yes")
			elif i==boneChain.size()-1:
				positionList.append(positionList[i]*2-positionList[i-1]) #extended last bone target
		PMpositionMatrix.append(positionList)
	
	#print("PMpositionMatrix",PMpositionMatrix)
	

func _get_bone_chain(skeleton, bone_index): #获取bone的所有子bone的index
	var bone_chain = [bone_index]  #第一个元素就是第一个bone的index
	var current_bone = bone_index
	while true:
		var child_bone = _get_single_child_bone(skeleton, current_bone) 
		if child_bone == -1: 
			break 
		bone_chain.append(child_bone) 
		current_bone = child_bone 
	return bone_chain 

func _get_single_child_bone(skeleton, bone_index): #获取bone的所有子bone的index
	for i in range(skeleton.get_bone_count()): 
		if skeleton.get_bone_parent(i) == bone_index: 
			return i 
	return -1

func _ready():
	if not skeleton: #search skeleton
		print("Skeleton3D not found")
		return
	for root_bone in rootBoneNameList: #create a list of bone chains
		var root_bone_index = skeleton.find_bone(root_bone)
		#print(root_bone_index)
		if root_bone_index == -1:
			print("Bone not found: ", root_bone)
			return
		_initialize_bone_chain(skeleton, root_bone_index) #add bone chains with their bone_id into boneIndexMatrix
	if !nodeDisabled:
		_prepare_positions()
		spawn_PM()
		link_PM()
		print("PM_list[0].size",PM_list[0].size())
		print("PMpositionMatrix.size",PMpositionMatrix[0].size())
		active=true
	
func spawn_PM():
	for i in PMpositionMatrix.size():
		PM_list.append([])
		for j in PMpositionMatrix[i].size():
			var new_PM
			var jRatio=float(j)/float(PMpositionMatrix[i].size())
			if j==0 and collideAtRoot==false:
				new_PM = nonCollisionPoint.new() #the first point does not need to be able to collide, lowering some cost
			else:
				new_PM = collisionPoint.new() #
				var derate:float=1
				if collider_size_curve:
					derate= collider_size_curve.sample(jRatio)
				new_PM.add_collider_with_size( collider_shape_radius_base * derate)
				new_PM.set_collision_layer(collider_collision_layer)
				new_PM.set_collision_mask(collider_collision_mask)
				if showColliderIndicator:
					new_PM.add_indicator_with_size( collider_shape_radius_base * derate)
			new_PM.name = "PointMass" + str(i) + str(j)
			new_PM.position = PMpositionMatrix[i][j]
			PM_list[i].append(new_PM)
			add_child(new_PM)
	

func _physics_process(delta):
	pin_to_bones(delta)
	for i in link_list.size():
		link_list[i].constrain()
		#link_list[i].constrain()
		#link_list[i].constrain()
	for i in PM_list.size():
		for j in PM_list[i].size():
			PM_list[i][j].do_verlet(delta)
	#print(Engine.get_frames_per_second())
	
	
func pin_to_bones(delta):  #pin the first point of each chain to the first bones
	#var mousepos = getargetVectoriewport().get_mouse_position()
	for i in PM_list.size(): 
		PM_list[i][0].is_pin=true
		PM_list[i][0].position = skeleton.get_bone_global_pose(boneIndexMatrix[i][0]).origin




##################################################################################################################################
#Rewriting Godot Official LookAtModifier3D for rotating each bone towards the colliders. 
#I tried to instance multiple official LookAtModifier3D to rotate the bones but it did not work

var target : Node3D
var bone_idx : int = 1
var forward_vector :Vector3
enum origin_from {
		ORIGIN_FROM_SELF,
		ORIGIN_FROM_SPECIFIC_BONE, #has to use this mode otherwise the rotation will be strange. except the first bone.
		ORIGIN_FROM_EXTERNAL_NODE,
	};

var ORIGIN_FROM_SPECIFIC_BONE :bool = true
var origin_bone :int = 1
var origin_offset :Vector3 = Vector3(0,0,0)
var ORIGIN_FROM_EXTERNAL_NODE :bool = false 
var forward_vector_nrm :Vector3
var primary_rotation_axis :Vector3 = Vector3(1,0,0)
var forward_axis :Vector3 = Vector3(0,1,0)
var secondary_rotation_axis: Vector3 = Vector3(0,0,1)
var use_angle_limitation :bool = false
var	from_q:Quaternion
var	prev_q:Quaternion
var	remaining :float = 0
var	time_step :float= 1.0
var origin_safe_margin :float =0
var is_within_limitations: bool = false
var symmetry_limitation: bool = true

var primary_limit_angle:float = TAU
var primary_damp_threshold : float = 1.0
var primary_negative_limit_angle:float= PI

var primary_negative_damp_threshold :float=1.0
var primary_positive_limit_angle:float= PI
var primary_positive_damp_threshold:float=1.0

var secondary_limit_angle:float = TAU
var secondary_negative_limit_angle:float= PI
var use_secondary_rotation :bool = true
var secondary_damp_threshold:float=1.0
var secondary_negative_damp_threshold:float=1.0
var secondary_positive_limit_angle:float= PI
var secondary_positive_damp_threshold:float=1.0

func _process_lookAt(bone:int, target:Node3D):
	var skeleton : Skeleton3D = get_skeleton()
	if (!skeleton || bone < 0 || bone >= skeleton.get_bone_count()):
		return
	# Calculate bone rest space in the world.
	var  bone_rest_space : Transform3D 
	var  parent_bone : int= skeleton.get_bone_parent(bone); #获取父级
	if (parent_bone < 0) : #如果没有父级骨，则用全局变换
		bone_rest_space = skeleton.get_global_transform();
		bone_rest_space.origin += skeleton.get_bone_rest(bone).origin;
	else :
		bone_rest_space = skeleton.get_global_transform() * skeleton.get_bone_global_pose(parent_bone);
		bone_rest_space.origin += skeleton.get_bone_rest(bone).origin;

	# Calculate forward_vector and destination.
	var prev_forward_vector :Vector3= forward_vector
	var destination:Quaternion
	origin_bone=bone #########################################very important
	if (!target) :#没有设置目标则不旋转
		destination = skeleton.get_bone_pose_rotation(bone);
	else:
		var origin_tr:Transform3D #
		if (origin_from.ORIGIN_FROM_SPECIFIC_BONE && origin_bone >= 0 && origin_bone < skeleton.get_bone_count()): #如果旋转原点是来自于指定骨
			origin_tr = skeleton.get_global_transform() * skeleton.get_bone_global_pose(origin_bone);
		elif (origin_from.ORIGIN_FROM_EXTERNAL_NODE) : #如果旋转原点是外部node
			var origin_src: Node3D = $""
			if (origin_src) :
				origin_tr = origin_src.get_global_transform()
			else:
				origin_tr = bone_rest_space
		else:#如果什么都没设置的话
			origin_tr = bone_rest_space
		forward_vector = (target.get_global_position() - origin_tr.translated_local(origin_offset).origin)*bone_rest_space.orthonormalized().basis
		forward_vector_nrm = forward_vector.normalized()
		if (forward_vector_nrm.abs().is_equal_approx(primary_rotation_axis)) :
			destination = skeleton.get_bone_pose_rotation(bone);
			forward_vector = Vector3(0, 0, 0); # The zero-vector to be used for checking in the line immediately below to avoid animation glitch.
		else:
			destination = look_at_with_axes(skeleton.get_bone_rest(bone)).basis.get_rotation_quaternion()

	# Detect flipping.
	var  is_not_max_influence: bool = influence < 1.0
	var  is_flippable : bool= use_angle_limitation || is_not_max_influence;
	var current_forward_axis :Vector3= forward_axis
	if (is_intersecting_axis(prev_forward_vector, forward_vector, primary_rotation_axis, primary_rotation_axis, true) ||
			(prev_forward_vector != Vector3(0, 0, 0) && forward_vector == Vector3(0, 0, 0)) ||
			(prev_forward_vector == Vector3(0, 0, 0) && forward_vector != Vector3(0, 0, 0))) :
		init_transition()
	elif (is_flippable && (get_value_from_same_axis(prev_forward_vector,secondary_rotation_axis))*(get_value_from_same_axis(forward_vector,secondary_rotation_axis))<0) :
		# Flipping by angle_limitation can be detected by sign of secondary rotation axes during forward_vector is rotated more than 90 degree from forward_axis (means dot production is negative).
		var prev_forward_vector_nrm: Vector3= forward_vector.normalized()
		var rest_forward_vector : Vector3= get_vector_from_bone_axis(forward_axis)
		if (symmetry_limitation):
			if ((is_not_max_influence || !is_equal_approx(primary_limit_angle, TAU)) &&
					prev_forward_vector_nrm.dot(rest_forward_vector) < 0 &&
					forward_vector_nrm.dot(rest_forward_vector) < 0):
				init_transition()
		else :
			if ((is_not_max_influence || !is_equal_approx(primary_positive_limit_angle + primary_negative_limit_angle, TAU)) &&
					prev_forward_vector_nrm.dot(rest_forward_vector) < 0 &&
					forward_vector_nrm.dot(rest_forward_vector) < 0):
				init_transition()
	skeleton.set_bone_pose_rotation(bone, destination);
	prev_q = destination;


func get_value_from_same_axis(a: Vector3, b: Vector3) -> float:
	if b.x == 1:
		return a.x
	elif b.y == 1:
		return a.y
	elif b.z == 1:
		return a.z
	else:
		return 0.0  # 如果b中没有1，则返回一个默认值


func init_transition():
	from_q = prev_q
	remaining = 1.0

	# For time-based interpolation.

func is_intersecting_axis( p_prev:Vector3, p_current,  p_flipping_axis :Vector3, p_check_axis :Vector3,  p_check_plane:bool) :
	# Prevent that the angular velocity does not become too large.
	# Check that is p_flipping_axis flipped nearby p_check_axis (close than origin_safe_margin) or not. If p_check_plane is true, check two axes of crossed plane.
	if (p_check_plane) :
		if (get_projection_vector(p_prev, p_check_axis).length() > origin_safe_margin   &&    get_projection_vector(p_current, p_check_axis).length() > origin_safe_margin):
			return false
	elif (abs(p_prev.distance_to(p_check_axis)) > origin_safe_margin &&	 abs(p_current.distance_to(p_check_axis)) > origin_safe_margin 	) :
		return false
	if p_prev.distance_to(p_flipping_axis) * p_current.distance_to(p_flipping_axis)<0:
		return true


func  get_vector_from_bone_axis(p_axis) ->Vector3:
	var ret:Vector3
	match (p_axis) :
		BONE_AXIS_PLUS_X: 
			ret = Vector3(1, 0, 0)
		BONE_AXIS_MINUS_X: 
			ret = Vector3(-1, 0, 0)
		BONE_AXIS_PLUS_Y: 
			ret = Vector3(0, 1, 0)
		BONE_AXIS_MINUS_Y: 
			ret = Vector3(0, -1, 0)
		BONE_AXIS_PLUS_Z: 
			ret = Vector3(0, 0, 1)
		BONE_AXIS_MINUS_Z: 
			ret = Vector3(0, 0, -1)
	return ret





func look_at_with_axes( p_rest:Transform3D) -> Transform3D:
	# Primary rotation by projection to 2D plane by xform_inv and picking elements.
	var current_vector : Vector3 = get_basis_vector_from_bone_axis(p_rest.basis, forward_axis).normalized()
	var src_vec2 :Vector2= get_projection_vector(forward_vector_nrm * p_rest.basis, primary_rotation_axis).normalized();
	var dst_vec2 :Vector2= get_projection_vector(current_vector * p_rest.basis, primary_rotation_axis).normalized();
	var calculated_angle :float= src_vec2.angle_to(dst_vec2)
	var primary_result :Transform3D= p_rest.rotated_local(primary_rotation_axis, calculated_angle);
	var current_result :Transform3D= primary_result; # primary_result will be used by calculation of secondary rotation, current_result is rotated by that.
	var limit_angle:float= 0.0;
	var damp_threshold :float = 0.0;

	if (use_angle_limitation) :
		if (symmetry_limitation) :
			limit_angle = primary_limit_angle * 0.5
			damp_threshold = primary_damp_threshold;
		else:
			if (calculated_angle<0) :
				limit_angle = primary_negative_limit_angle;
				damp_threshold = primary_negative_damp_threshold;
			else:
				limit_angle = primary_positive_limit_angle;
				damp_threshold = primary_positive_damp_threshold;
		if (abs(calculated_angle) > limit_angle) :
			is_within_limitations = false
		calculated_angle = remap_damped(0, limit_angle, damp_threshold, calculated_angle);
		current_result = p_rest.rotated_local(primary_rotation_axis, calculated_angle);
	#Needs for detecting flipping even if use_secondary_rotation is false.
	secondary_rotation_axis = get_secondary_rotation_axis(forward_axis, primary_rotation_axis);
	if (!use_secondary_rotation):
		return current_result
	#Secondary rotation by projection to 2D plane by xform_inv and picking elements.
	current_vector = get_basis_vector_from_bone_axis(primary_result.basis, forward_axis).normalized();
	src_vec2 = get_projection_vector(forward_vector_nrm * primary_result.basis, secondary_rotation_axis).normalized();
	dst_vec2 = get_projection_vector(current_vector * primary_result.basis, secondary_rotation_axis).normalized();
	calculated_angle = src_vec2.angle_to(dst_vec2);
	if (use_angle_limitation):
		if (symmetry_limitation) :
			limit_angle = secondary_limit_angle * 0.5
			damp_threshold = secondary_damp_threshold;
		else:
			if (calculated_angle<0):
				limit_angle = secondary_negative_limit_angle;
				damp_threshold = secondary_negative_damp_threshold;
			else:
				limit_angle = secondary_positive_limit_angle;
				damp_threshold = secondary_positive_damp_threshold;
		if (abs(calculated_angle) > limit_angle) :
			is_within_limitations = false
		calculated_angle = remap_damped(0, limit_angle, damp_threshold, calculated_angle)
	current_result = current_result.rotated_local(secondary_rotation_axis, calculated_angle)
	return current_result


func get_projection_vector(p_vector : Vector3, p_axis:Vector3) -> Vector2:
	# NOTE: axis is swapped between 2D and 3D.
	var ret: Vector2
	match (p_axis) :
		Vector3(1,0,0): 
			ret = Vector2(p_vector.z, p_vector.y);
		Vector3(0,1,0):
			ret = Vector2(p_vector.x, p_vector.z);
		Vector3(0,0,1):
			ret = Vector2(p_vector.y, p_vector.x);
	return ret;


func get_basis_vector_from_bone_axis(p_basis,  p_axis) ->Vector3:
	var ret :Vector3;
	match (p_axis) :
		Vector3(1,0,0):
			ret = p_basis.x
		Vector3(-1,0,0):
			ret = -p_basis.x
		Vector3(0,1,0):
			ret = p_basis.y
		Vector3(0,-1,0):
			ret = -p_basis.y
		Vector3(0,0,1):
			ret = p_basis.z
		Vector3(0,0,-1):
			ret = -p_basis.z
	return ret;




func remap_damped(p_from: float, p_to: float, p_damp_threshold: float, p_value: float) -> float: 
	var sign_flag
	if p_value<0:
		sign_flag=-1
	else:
		sign_flag=1
	var abs_value :float = abs(p_value);
	if (is_equal_approx(p_damp_threshold, 1.0) || is_zero_approx(p_to)) :
		return sign * clamp(abs_value,p_from, p_to); #Avoid division by zero.
	var value :float = inverse_lerp(p_from, p_to, abs_value)
	if value <= p_damp_threshold :
		return sign * clamp(abs_value, p_from, p_to)
	var limit : float= PI
	var inv_to : float = 1.0 / p_to;
	var end_x : float = limit * inv_to;
	var _position :float= abs_value * inv_to;
	var start : Vector2= Vector2(p_damp_threshold, p_damp_threshold);
	var mid : Vector2= Vector2(1.0, 1.0);
	var end : Vector2= Vector2(end_x, 1.0);
	value = get_bspline_y(start, mid, end, _position);
	return sign_flag * lerp(p_from, p_to, value);


func get_secondary_rotation_axis(p_forward_axis: Vector3,  p_primary_rotation_axis: Vector3) ->Vector3:
	var secondary_plane : Vector3= p_forward_axis + p_primary_rotation_axis
# Determine the secondary_plane vector
# Check if the x component of secondary_plane is approximately zero
	if is_zero_approx(secondary_plane.x):
		return Vector3(1,0,0)
	else:
		# Check if the y component of secondary_plane is approximately zero
		if is_zero_approx(secondary_plane.y):
			return Vector3(0,1,0)
		else:
			# Default to the z axis
			return Vector3(0,0,1)


func get_bspline_y(p_from : Vector2, p_control :Vector2, p_to: Vector2,  p_x:float) -> float:
	var a :float = p_from.x - 2.0 * p_control.x + p_to.x
	var b :float= -2.0 * p_from.x + 2.0 * p_control.x
	var c :float= p_from.x - p_x
	var t :float= 0.0
	if is_zero_approx(a) :
		t = -c / b #Almost linear.
	else:
		var discriminant :float = b * b - 4.0 * a * c
		var sqrt_discriminant :float = sqrt(discriminant)
		var e :float= 1.0 / (2.0 * a)
		var t1 :float= (-b + sqrt_discriminant) * e
		if (0.0 <= t1 && t1 <= 1.0):
			t=t1
		else:
			t= (-b - sqrt_discriminant) * e
	var u :float = 1.0 - t;
	var y :float = u * u * p_from.y + 2.0 * u * t * p_control.y + t * t * p_to.y;
	return y


























"
Supported Customisability In Area:

- grid_size: determines the number of point masses on the cloth
- PM_spacing: determines the space between each point mass

Supported Customisability In Link:
Note that a link can connect ANY two Godot nodes, though only my point masses will experience verlet physics (have continued momentum).

- is_rigid: if a link is rigid it will resist compression. The cloth links do not resist compression, but this will allow you to create rigid objects
- is_PM_a_pin/is_PM_b_pin: A pin node will not be pulled by a link. By default any node you connect to a link will be a pin. This can be overriden by adding a variable to your node called 'is_pin'
Supported Customisability In PointMass:
- acceleration: 'accelerates' the mass constantly in this direction. By default this is used as gravity, but it could also implement wind and other forces.
- dampen_factor: multiplies velocity by this factor each frame
- collision_factor: This is a value I had to tweak to make collision feel right. Increase it and the mass will become more bouncy and experience more friction. If it is set too low, high speed masses will stick to walls instead of bouncing.
Performance:

- I don't recommend you make a cloth much bigger than 7x7 if you want a consistent 60fps on low-end machines. This should be plenty enough imo, you can still make the links longer if you want a bigger cloth.
- You can improve performance by quite a bit by making the constrain() function run less times per frame. By default it runs 3 times per frame. In short, running it several times stops the cloth from 'shivering.' The shivering isn't super noticeable so it wouldn't hurt to reduce this.
- You could also maybe just half the rate at which Area updates its physics, which in theory would make it twice as performant.
TODO & Contributions:
I may add support for adjusting the rigidity of the links and giving the cloth an outline.
Feel free to make suggestions, or add your own features to this repo. The code should be fairly easy to understand I think."
