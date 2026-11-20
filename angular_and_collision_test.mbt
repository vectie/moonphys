///|
test "angular acceleration uses diagonal inertia" {
  let inertia = @moonphys.rigid_body_inertia(
    "body",
    @moonphys.vec3(2.0, 4.0, 8.0),
  )
  let acceleration = @moonphys.angular_acceleration(
    inertia,
    @moonphys.vec3(4.0, 4.0, 4.0),
  )
  assert_true(acceleration == @moonphys.vec3(2.0, 1.0, 0.5))
}

///|
test "hinge axis assessment accepts aligned and reversed axes" {
  let constraint = @moonphys.rigid_body_hinge_axis_constraint(
    "knee-hinge",
    "upper",
    "lower",
    local_axis_a=@moonphys.vec3(0.0, 1.0, 0.0),
    local_axis_b=@moonphys.vec3(0.0, -1.0, 0.0),
  )
  let upper = @moonphys.rigid_body_angular_state(
    "upper",
    @moonphys.identity_basis3(),
    @moonphys.vec3(0.0, 0.0, 0.0),
  )
  let lower = @moonphys.rigid_body_angular_state(
    "lower",
    @moonphys.identity_basis3(),
    @moonphys.vec3(0.0, 2.0, 0.0),
  )
  let assessment = @moonphys.rigid_body_hinge_axis_assessment(
    constraint, upper, lower,
  )
  assert_eq(assessment.status, "hinge-axis-spinning")
  assert_true(nearly_equal(assessment.angular_error_rad, 0.0))
  assert_true(nearly_equal(assessment.alignment_dot, 1.0))
  assert_true(nearly_equal(assessment.relative_axis_velocity_rad_s, 2.0))
  assert_true(assessment.world_axis_a == @moonphys.vec3(0.0, 1.0, 0.0))
  assert_true(assessment.world_axis_b == @moonphys.vec3(0.0, 1.0, 0.0))
}

///|
test "hinge axis assessment reports angular misalignment" {
  let constraint = @moonphys.rigid_body_hinge_axis_constraint(
    "hip-hinge",
    "pelvis",
    "thigh",
    local_axis_a=@moonphys.vec3(0.0, 1.0, 0.0),
    local_axis_b=@moonphys.vec3(0.0, 1.0, 0.0),
    stiffness=0.5,
  )
  let pelvis = @moonphys.rigid_body_angular_state(
    "pelvis",
    @moonphys.identity_basis3(),
    @moonphys.vec3(0.0, 0.0, 0.0),
  )
  let thigh = @moonphys.rigid_body_angular_state(
    "thigh",
    @moonphys.axis_angle_basis3(
      @moonphys.vec3(0.0, 0.0, 1.0),
      3.141592653589793 / 2.0,
    ),
    @moonphys.vec3(0.0, 0.0, 0.0),
  )
  let assessment = @moonphys.rigid_body_hinge_axis_assessment(
    constraint, pelvis, thigh,
  )
  assert_eq(assessment.status, "hinge-axis-misaligned")
  assert_true(nearly_equal(assessment.alignment_dot, 0.0))
  assert_true(
    nearly_equal(assessment.angular_error_rad, 3.141592653589793 / 2.0),
  )
  assert_true(
    nearly_equal(assessment.correction_angle_rad, 3.141592653589793 / 4.0),
  )
  assert_true(nearly_equal(assessment.correction_axis.length(), 1.0))
  assert_true(
    nearly_equal(
      if assessment.correction_axis.z < 0.0 {
        0.0 - assessment.correction_axis.z
      } else {
        assessment.correction_axis.z
      },
      1.0,
    ),
  )
}

///|
test "hinge axis correction splits angular error by inverse inertia" {
  let constraint = @moonphys.rigid_body_hinge_axis_constraint(
    "hip-hinge",
    "pelvis",
    "thigh",
    local_axis_a=@moonphys.vec3(0.0, 1.0, 0.0),
    local_axis_b=@moonphys.vec3(0.0, 1.0, 0.0),
  )
  let pelvis = @moonphys.rigid_body_angular_state(
    "pelvis",
    @moonphys.identity_basis3(),
    @moonphys.vec3(0.0, 0.0, 0.0),
  )
  let thigh = @moonphys.rigid_body_angular_state(
    "thigh",
    @moonphys.axis_angle_basis3(
      @moonphys.vec3(0.0, 0.0, 1.0),
      3.141592653589793 / 2.0,
    ),
    @moonphys.vec3(0.0, 0.0, 0.0),
  )
  let correction = @moonphys.rigid_body_hinge_axis_correction(
    constraint,
    pelvis,
    @moonphys.rigid_body_inertia("pelvis", @moonphys.vec3(1.0, 1.0, 1.0)),
    thigh,
    @moonphys.rigid_body_inertia("thigh", @moonphys.vec3(3.0, 3.0, 3.0)),
  )
  assert_eq(correction.status, "hinge-correction-estimated")
  assert_true(nearly_equal(correction.inverse_inertia_a, 1.0))
  assert_true(nearly_equal(correction.inverse_inertia_b, 1.0 / 3.0))
  assert_true(
    nearly_equal(
      correction.correction_a_rad.length(),
      3.141592653589793 * 0.375,
    ),
  )
  assert_true(
    nearly_equal(
      correction.correction_b_rad.length(),
      3.141592653589793 * 0.125,
    ),
  )
  assert_true(nearly_equal(correction.angular_impulse_nms, 0.0))
}

///|
test "hinge axis correction estimates velocity impulse" {
  let constraint = @moonphys.rigid_body_hinge_axis_constraint(
    "knee-hinge",
    "upper",
    "lower",
    local_axis_a=@moonphys.vec3(0.0, 1.0, 0.0),
    local_axis_b=@moonphys.vec3(0.0, 1.0, 0.0),
  )
  let upper = @moonphys.rigid_body_angular_state(
    "upper",
    @moonphys.identity_basis3(),
    @moonphys.vec3(0.0, 0.0, 0.0),
  )
  let lower = @moonphys.rigid_body_angular_state(
    "lower",
    @moonphys.identity_basis3(),
    @moonphys.vec3(0.0, 4.0, 0.0),
  )
  let correction = @moonphys.rigid_body_hinge_axis_correction(
    constraint,
    upper,
    @moonphys.rigid_body_inertia("upper", @moonphys.vec3(2.0, 2.0, 2.0)),
    lower,
    @moonphys.rigid_body_inertia("lower", @moonphys.vec3(2.0, 2.0, 2.0)),
  )
  assert_eq(correction.status, "hinge-correction-estimated")
  assert_true(nearly_equal(correction.angular_impulse_nms, 4.0))
  assert_true(nearly_equal(correction.velocity_delta_a_rad_s.y, 2.0))
  assert_true(nearly_equal(correction.velocity_delta_b_rad_s.y, -2.0))
}

///|
test "hinge joint assessment composes anchor and axis evidence" {
  let constraint = @moonphys.rigid_body_hinge_joint_constraint(
    "hip",
    "pelvis",
    "thigh",
    anchor_a_m=@moonphys.vec3(1.0, 0.0, 0.0),
    anchor_b_m=@moonphys.vec3(0.0 - 1.0, 0.0, 0.0),
    local_axis_a=@moonphys.vec3(0.0, 1.0, 0.0),
    local_axis_b=@moonphys.vec3(0.0, 1.0, 0.0),
  )
  let pelvis = @moonphys.rigid_body_motion_state(
    "pelvis",
    2.0,
    0.1,
    @moonphys.vec3(0.0, 0.0, 2.0),
    @moonphys.vec3(0.0, 0.0, 0.0),
    @moonphys.identity_basis3(),
    @moonphys.vec3(0.0, 0.0, 0.0),
  )
  let thigh = @moonphys.rigid_body_motion_state(
    "thigh",
    2.0,
    0.1,
    @moonphys.vec3(3.0, 0.0, 2.0),
    @moonphys.vec3(0.0, 0.0, 0.0),
    @moonphys.axis_angle_basis3(
      @moonphys.vec3(0.0, 0.0, 1.0),
      3.141592653589793 / 2.0,
    ),
    @moonphys.vec3(0.0, 0.0, 0.0),
  )
  let assessment = @moonphys.rigid_body_hinge_joint_assessment(
    constraint,
    pelvis,
    @moonphys.rigid_body_inertia("pelvis", @moonphys.vec3(2.0, 2.0, 2.0)),
    thigh,
    @moonphys.rigid_body_inertia("thigh", @moonphys.vec3(2.0, 2.0, 2.0)),
  )
  assert_eq(assessment.status, "hinge-joint-correction-estimated")
  assert_eq(assessment.position.status, "world-constraint-resolved")
  assert_eq(assessment.axis.status, "hinge-correction-estimated")
  assert_true(nearly_equal(assessment.position_error_m, 1.0))
  assert_true(
    nearly_equal(assessment.angular_error_rad, 3.141592653589793 / 2.0),
  )
  assert_true(nearly_equal(assessment.position.world_anchor_a.x, 1.0))
  assert_true(nearly_equal(assessment.position.world_anchor_b.x, 2.0))
}

///|
test "hinge joint frame assessment aggregates joints" {
  let root = @moonphys.rigid_body_motion_state(
    "root",
    2.0,
    0.1,
    @moonphys.vec3(0.0, 0.0, 2.0),
    @moonphys.vec3(0.0, 0.0, 0.0),
    @moonphys.identity_basis3(),
    @moonphys.vec3(0.0, 0.0, 0.0),
  )
  let upper = @moonphys.rigid_body_motion_state(
    "upper",
    2.0,
    0.1,
    @moonphys.vec3(2.0, 0.0, 2.0),
    @moonphys.vec3(0.0, 0.0, 0.0),
    @moonphys.identity_basis3(),
    @moonphys.vec3(0.0, 0.0, 0.0),
  )
  let lower = @moonphys.rigid_body_motion_state(
    "lower",
    2.0,
    0.1,
    @moonphys.vec3(4.0, 0.0, 2.0),
    @moonphys.vec3(0.0, 0.0, 0.0),
    @moonphys.axis_angle_basis3(
      @moonphys.vec3(0.0, 0.0, 1.0),
      3.141592653589793 / 2.0,
    ),
    @moonphys.vec3(0.0, 0.0, 0.0),
  )
  let frame = @moonphys.rigid_body_hinge_joint_frame_assessment(
    "leg-frame",
    [
      @moonphys.rigid_body_joint_frame_body(
        root,
        @moonphys.rigid_body_inertia("root", @moonphys.vec3(2.0, 2.0, 2.0)),
      ),
      @moonphys.rigid_body_joint_frame_body(
        upper,
        @moonphys.rigid_body_inertia("upper", @moonphys.vec3(2.0, 2.0, 2.0)),
      ),
      @moonphys.rigid_body_joint_frame_body(
        lower,
        @moonphys.rigid_body_inertia("lower", @moonphys.vec3(2.0, 2.0, 2.0)),
      ),
    ],
    [
      @moonphys.rigid_body_hinge_joint_constraint(
        "hip",
        "root",
        "upper",
        anchor_a_m=@moonphys.vec3(1.0, 0.0, 0.0),
        anchor_b_m=@moonphys.vec3(-1.0, 0.0, 0.0),
        local_axis_a=@moonphys.vec3(0.0, 1.0, 0.0),
        local_axis_b=@moonphys.vec3(0.0, 1.0, 0.0),
      ),
      @moonphys.rigid_body_hinge_joint_constraint(
        "knee",
        "upper",
        "lower",
        anchor_a_m=@moonphys.vec3(1.0, 0.0, 0.0),
        anchor_b_m=@moonphys.vec3(-1.0, 0.0, 0.0),
        local_axis_a=@moonphys.vec3(0.0, 1.0, 0.0),
        local_axis_b=@moonphys.vec3(0.0, 1.0, 0.0),
      ),
    ],
  )
  assert_eq(frame.status, "hinge-joint-frame-correction-estimated")
  assert_eq(frame.body_count, 3)
  assert_eq(frame.joint_count, 2)
  assert_eq(frame.assessed_joint_count, 2)
  assert_eq(frame.review_count, 0)
  assert_true(nearly_equal(frame.max_position_error_m, 0.0))
  assert_true(
    nearly_equal(frame.max_angular_error_rad, 3.141592653589793 / 2.0),
  )
  assert_eq(frame.joints[0].status, "hinge-joint-satisfied")
  assert_eq(frame.joints[1].status, "hinge-joint-correction-estimated")
}

///|
test "primitive diagonal inertia uses solid shape formulas" {
  let sphere = @moonphys.sphere_diagonal_inertia("sphere", 10.0, 2.0)
  assert_true(nearly_equal(sphere.diagonal_kg_m2.x, 16.0))
  assert_true(nearly_equal(sphere.diagonal_kg_m2.y, 16.0))
  assert_true(nearly_equal(sphere.diagonal_kg_m2.z, 16.0))
  let box = @moonphys.box_diagonal_inertia(
    "box",
    12.0,
    @moonphys.vec3(1.0, 2.0, 3.0),
  )
  assert_true(nearly_equal(box.diagonal_kg_m2.x, 52.0))
  assert_true(nearly_equal(box.diagonal_kg_m2.y, 40.0))
  assert_true(nearly_equal(box.diagonal_kg_m2.z, 20.0))
  let capsule = @moonphys.capsule_diagonal_inertia("capsule", 10.0, 1.0, 1.0)
  assert_true(nearly_equal(capsule.diagonal_kg_m2.x, 12.1))
  assert_true(nearly_equal(capsule.diagonal_kg_m2.y, 12.1))
  assert_true(nearly_equal(capsule.diagonal_kg_m2.z, 4.6))
}

///|
test "collision shape diagonal inertia dispatches by primitive kind" {
  let inertia = @moonphys.collision_shape_diagonal_inertia(
    "body",
    12.0,
    @moonphys.collision_box("body-box", @moonphys.vec3(1.0, 2.0, 3.0)),
  )
  assert_true(nearly_equal(inertia.diagonal_kg_m2.x, 52.0))
  assert_true(nearly_equal(inertia.diagonal_kg_m2.y, 40.0))
  assert_true(nearly_equal(inertia.diagonal_kg_m2.z, 20.0))
}

///|
test "collision shape volume dispatches by primitive kind" {
  assert_true(
    nearly_equal(
      @moonphys.collision_shape_volume(
        @moonphys.collision_sphere("sphere", 1.0),
      ),
      4.0 * 3.141592653589793 / 3.0,
    ),
  )
  assert_true(
    nearly_equal(
      @moonphys.collision_shape_volume(
        @moonphys.collision_box("box", @moonphys.vec3(1.0, 2.0, 3.0)),
      ),
      48.0,
    ),
  )
  assert_true(
    @moonphys.collision_shape_volume(
      @moonphys.collision_capsule("capsule", 1.0, 1.0),
    ) >
    @moonphys.collision_shape_volume(@moonphys.collision_sphere("sphere", 1.0)),
  )
}

///|
test "composite mass properties apply parallel axis theorem" {
  let composite = @moonphys.composite_mass_properties("body", [
    @moonphys.mass_property_element(
      "left",
      1.0,
      @moonphys.vec3(0.0, 0.0, 0.0),
      @moonphys.rigid_body_inertia("left", @moonphys.vec3(0.0, 0.0, 0.0)),
    ),
    @moonphys.mass_property_element(
      "right",
      3.0,
      @moonphys.vec3(2.0, 0.0, 0.0),
      @moonphys.rigid_body_inertia("right", @moonphys.vec3(0.0, 0.0, 0.0)),
    ),
  ])
  assert_eq(composite.element_count, 2)
  assert_true(nearly_equal(composite.total_mass_kg, 4.0))
  assert_true(nearly_equal(composite.center_of_mass.x, 1.5))
  assert_true(nearly_equal(composite.center_of_mass.y, 0.0))
  assert_true(
    nearly_equal(composite.inertia_about_center.diagonal_kg_m2.x, 0.0),
  )
  assert_true(
    nearly_equal(composite.inertia_about_center.diagonal_kg_m2.y, 3.0),
  )
  assert_true(
    nearly_equal(composite.inertia_about_center.diagonal_kg_m2.z, 3.0),
  )
  assert_eq(composite.status, "composite-mass-resolved")
}

///|
test "collision shape mass element contributes primitive inertia" {
  let element = @moonphys.collision_shape_mass_element(
    "box",
    12.0,
    @moonphys.collision_box("box-shape", @moonphys.vec3(1.0, 2.0, 3.0)),
    @moonphys.vec3(0.0, 0.0, 0.0),
  )
  assert_true(nearly_equal(element.inertia_about_center.diagonal_kg_m2.x, 52.0))
  assert_true(nearly_equal(element.inertia_about_center.diagonal_kg_m2.y, 40.0))
  assert_true(nearly_equal(element.inertia_about_center.diagonal_kg_m2.z, 20.0))
}

///|
test "angular rigid body integration applies torque semi implicitly" {
  let inertia = @moonphys.rigid_body_inertia(
    "body",
    @moonphys.vec3(2.0, 2.0, 2.0),
  )
  let state = @moonphys.rigid_body_angular_state(
    "body",
    @moonphys.identity_basis3(),
    @moonphys.vec3(0.0, 0.0, 0.0),
  )
  let stepped = @moonphys.integrate_angular_rigid_body(state, inertia, {
    dt_s: 0.5,
    torque_nm: @moonphys.vec3(0.0, 0.0, 4.0),
  })
  let rotated = stepped.orientation.mul_vec3(@moonphys.vec3(1.0, 0.0, 0.0))
  assert_true(stepped.angular_velocity_rad_s == @moonphys.vec3(0.0, 0.0, 1.0))
  assert_true(nearly_equal(rotated.x, @math.cos(0.5)))
  assert_true(nearly_equal(rotated.y, @math.sin(0.5)))
}

///|
test "rigid body wrench integration combines force and torque" {
  let state = @moonphys.rigid_body_motion_state(
    "body",
    2.0,
    0.0,
    @moonphys.vec3(0.0, 0.0, 0.0),
    @moonphys.vec3(0.0, 0.0, 0.0),
    @moonphys.identity_basis3(),
    @moonphys.vec3(0.0, 0.0, 0.0),
  )
  let step = @moonphys.integrate_rigid_body_wrench(
    state,
    @moonphys.rigid_body_inertia("body", @moonphys.vec3(1.0, 2.0, 4.0)),
    @moonphys.rigid_body_wrench(
      @moonphys.vec3(4.0, 0.0, 0.0),
      @moonphys.vec3(0.0, 4.0, 8.0),
    ),
    0.5,
  )
  assert_eq(step.status, "wrench-integrated")
  assert_true(step.linear_acceleration_mps2 == @moonphys.vec3(2.0, 0.0, 0.0))
  assert_true(step.angular_acceleration_rad_s2 == @moonphys.vec3(0.0, 2.0, 2.0))
  assert_true(step.linear_impulse_ns == @moonphys.vec3(2.0, 0.0, 0.0))
  assert_true(step.angular_impulse_nms == @moonphys.vec3(0.0, 2.0, 4.0))
  assert_true(step.after.linear.velocity == @moonphys.vec3(1.0, 0.0, 0.0))
  assert_true(step.after.linear.position == @moonphys.vec3(0.5, 0.0, 0.0))
  assert_true(
    step.after.angular.angular_velocity_rad_s == @moonphys.vec3(0.0, 1.0, 1.0),
  )
  assert_true(nearly_equal(step.kinetic_energy_before_j, 0.0))
  assert_true(nearly_equal(step.kinetic_energy_after_j, 4.0))
  assert_true(nearly_equal(step.kinetic_energy_delta_j, 4.0))
}

///|
test "rigid body wrench integration flags zero mass review" {
  let state = @moonphys.rigid_body_motion_state(
    "body",
    0.0,
    0.0,
    @moonphys.vec3(0.0, 0.0, 0.0),
    @moonphys.vec3(0.0, 0.0, 0.0),
    @moonphys.identity_basis3(),
    @moonphys.vec3(0.0, 0.0, 0.0),
  )
  let step = @moonphys.integrate_rigid_body_wrench(
    state,
    @moonphys.rigid_body_inertia("body", @moonphys.vec3(1.0, 1.0, 1.0)),
    @moonphys.rigid_body_wrench(
      @moonphys.vec3(4.0, 0.0, 0.0),
      @moonphys.vec3(0.0, 0.0, 0.0),
    ),
    0.5,
  )
  assert_eq(step.status, "wrench-zero-mass-review")
  assert_true(step.linear_acceleration_mps2 == @moonphys.vec3(0.0, 0.0, 0.0))
  assert_true(step.linear_impulse_ns == @moonphys.vec3(2.0, 0.0, 0.0))
  assert_true(nearly_equal(step.kinetic_energy_delta_j, 0.0))
  assert_true(step.after.linear.position == @moonphys.vec3(0.0, 0.0, 0.0))
}

///|
test "angular momentum and contact torque are deterministic" {
  let inertia = @moonphys.rigid_body_inertia(
    "body",
    @moonphys.vec3(2.0, 4.0, 8.0),
  )
  let state = @moonphys.rigid_body_angular_state(
    "body",
    @moonphys.identity_basis3(),
    @moonphys.vec3(1.0, 2.0, 3.0),
  )
  assert_true(
    @moonphys.angular_momentum(inertia, state) == @moonphys.vec3(2.0, 8.0, 24.0),
  )
  assert_true(
    @moonphys.contact_torque(
      @moonphys.vec3(0.0, 0.0, 0.0),
      @moonphys.vec3(0.0, 1.0, 0.0),
      @moonphys.vec3(1.0, 0.0, 0.0),
    ) ==
    @moonphys.vec3(0.0, 0.0, -1.0),
  )
}

///|
test "collision shapes expose conservative bounding radii" {
  assert_eq(@moonphys.collision_shape_kind_label(SphereShape), "sphere")
  assert_true(
    nearly_equal(
      @moonphys.collision_bounding_radius(@moonphys.collision_sphere("s", 0.5)),
      0.5,
    ),
  )
  assert_true(
    nearly_equal(
      @moonphys.collision_bounding_radius(
        @moonphys.collision_capsule("c", 0.2, 1.0),
      ),
      1.2,
    ),
  )
  assert_true(
    nearly_equal(
      @moonphys.collision_bounding_radius(
        @moonphys.collision_box("b", @moonphys.vec3(1.0, 2.0, 2.0)),
      ),
      3.0,
    ),
  )
}

///|
test "collision world bounds follow transform translation" {
  let bounds = @moonphys.collision_world_bounds(
    @moonphys.collision_sphere("s", 0.5),
    @moonphys.transform3(
      @moonphys.identity_basis3(),
      @moonphys.vec3(1.0, 2.0, 3.0),
    ),
  )
  assert_true(bounds.center == @moonphys.vec3(1.0, 2.0, 3.0))
  assert_true(bounds.min == @moonphys.vec3(0.5, 1.5, 2.5))
  assert_true(bounds.max == @moonphys.vec3(1.5, 2.5, 3.5))
  assert_eq(bounds.status, "conservative-bounds")
}

///|
test "collision pair probe reports broadphase overlap and clear" {
  let shape = @moonphys.collision_sphere("a", 0.5)
  let overlap = @moonphys.collision_pair_probe(
    shape,
    @moonphys.transform3(
      @moonphys.identity_basis3(),
      @moonphys.vec3(0.0, 0.0, 0.0),
    ),
    @moonphys.collision_sphere("b", 0.5),
    @moonphys.transform3(
      @moonphys.identity_basis3(),
      @moonphys.vec3(0.75, 0.0, 0.0),
    ),
  )
  let clear = @moonphys.collision_pair_probe(
    shape,
    @moonphys.transform3(
      @moonphys.identity_basis3(),
      @moonphys.vec3(0.0, 0.0, 0.0),
    ),
    @moonphys.collision_sphere("b", 0.5),
    @moonphys.transform3(
      @moonphys.identity_basis3(),
      @moonphys.vec3(2.0, 0.0, 0.0),
    ),
  )
  assert_true(overlap.overlapping)
  assert_true(nearly_equal(overlap.separation_m, -0.25))
  assert_eq(overlap.status, "broadphase-overlap")
  assert_true(!clear.overlapping)
  assert_true(nearly_equal(clear.separation_m, 1.0))
  assert_eq(clear.status, "broadphase-clear")
}

///|
test "sphere sphere contact reports exact penetration" {
  let contact = @moonphys.sphere_sphere_contact(
    @moonphys.collision_sphere("a", 0.5),
    @moonphys.transform3(
      @moonphys.identity_basis3(),
      @moonphys.vec3(0.0, 0.0, 0.0),
    ),
    @moonphys.collision_sphere("b", 0.75),
    @moonphys.transform3(
      @moonphys.identity_basis3(),
      @moonphys.vec3(1.0, 0.0, 0.0),
    ),
  )
  assert_true(contact.in_contact)
  assert_true(nearly_equal(contact.separation_m, -0.25))
  assert_true(nearly_equal(contact.penetration_depth_m, 0.25))
  assert_true(contact.normal == @moonphys.vec3(1.0, 0.0, 0.0))
  assert_true(nearly_equal(contact.point.x, 0.375))
  assert_eq(contact.status, "sphere-sphere-contact")
}

///|
test "collision pair contact uses narrow phase for non spheres" {
  let sphere_capsule = @moonphys.collision_pair_contact(
    @moonphys.collision_sphere("sphere", 0.25),
    @moonphys.transform3(
      @moonphys.identity_basis3(),
      @moonphys.vec3(0.0, 0.0, 0.0),
    ),
    @moonphys.collision_capsule("capsule", 0.2, 0.5),
    @moonphys.transform3(
      @moonphys.identity_basis3(),
      @moonphys.vec3(0.4, 0.0, 0.0),
    ),
  )
  assert_true(sphere_capsule.in_contact)
  assert_eq(sphere_capsule.status, "sphere-capsule-contact")
  let capsule_capsule = @moonphys.collision_pair_contact(
    @moonphys.collision_capsule("a", 0.2, 0.5),
    @moonphys.transform3(
      @moonphys.identity_basis3(),
      @moonphys.vec3(0.0, 0.0, 0.0),
    ),
    @moonphys.collision_capsule("b", 0.2, 0.5),
    @moonphys.transform3(
      @moonphys.identity_basis3(),
      @moonphys.vec3(0.35, 0.0, 0.0),
    ),
  )
  assert_true(capsule_capsule.in_contact)
  assert_eq(capsule_capsule.status, "capsule-capsule-contact")
  let capsule_box = @moonphys.collision_pair_contact(
    @moonphys.collision_capsule("capsule", 0.2, 0.5),
    @moonphys.transform3(
      @moonphys.identity_basis3(),
      @moonphys.vec3(0.0, 0.0, 0.0),
    ),
    @moonphys.collision_box("box", @moonphys.vec3(0.3, 0.3, 0.3)),
    @moonphys.transform3(
      @moonphys.identity_basis3(),
      @moonphys.vec3(0.45, 0.0, 0.0),
    ),
  )
  assert_true(capsule_box.in_contact)
  assert_eq(capsule_box.status, "capsule-box-contact")
}

///|
test "box box narrow phase rejects bounding sphere false positive" {
  let clear = @moonphys.collision_pair_contact(
    @moonphys.collision_box("a", @moonphys.vec3(1.0, 1.0, 0.1)),
    @moonphys.transform3(
      @moonphys.identity_basis3(),
      @moonphys.vec3(0.0, 0.0, 0.0),
    ),
    @moonphys.collision_box("b", @moonphys.vec3(1.0, 1.0, 0.1)),
    @moonphys.transform3(
      @moonphys.identity_basis3(),
      @moonphys.vec3(2.1, 0.0, 0.0),
    ),
  )
  assert_true(!clear.in_contact)
  assert_eq(clear.status, "box-box-clear")
  assert_true(nearly_equal(clear.separation_m, 0.1))
  let rotated = @moonphys.collision_pair_contact(
    @moonphys.collision_box("a", @moonphys.vec3(1.0, 0.25, 0.25)),
    @moonphys.transform3(
      @moonphys.yaw_basis3(3.141592653589793 / 4.0),
      @moonphys.vec3(0.0, 0.0, 0.0),
    ),
    @moonphys.collision_box("b", @moonphys.vec3(0.5, 0.5, 0.5)),
    @moonphys.transform3(
      @moonphys.identity_basis3(),
      @moonphys.vec3(0.75, 0.0, 0.0),
    ),
  )
  assert_true(rotated.in_contact)
  assert_eq(rotated.status, "box-box-contact")
  assert_true(rotated.penetration_depth_m > 0.0)
}

///|
test "collision contact manifold summarizes active contacts" {
  let first = @moonphys.sphere_sphere_contact(
    @moonphys.collision_sphere("a", 0.5),
    @moonphys.transform3(
      @moonphys.identity_basis3(),
      @moonphys.vec3(0.0, 0.0, 0.0),
    ),
    @moonphys.collision_sphere("b", 0.5),
    @moonphys.transform3(
      @moonphys.identity_basis3(),
      @moonphys.vec3(0.75, 0.0, 0.0),
    ),
  )
  let second = @moonphys.sphere_sphere_contact(
    @moonphys.collision_sphere("a", 0.5),
    @moonphys.transform3(
      @moonphys.identity_basis3(),
      @moonphys.vec3(0.0, 0.0, 0.0),
    ),
    @moonphys.collision_sphere("c", 0.5),
    @moonphys.transform3(
      @moonphys.identity_basis3(),
      @moonphys.vec3(0.0, 0.75, 0.0),
    ),
  )
  let clear = @moonphys.sphere_sphere_contact(
    @moonphys.collision_sphere("a", 0.5),
    @moonphys.transform3(
      @moonphys.identity_basis3(),
      @moonphys.vec3(0.0, 0.0, 0.0),
    ),
    @moonphys.collision_sphere("d", 0.5),
    @moonphys.transform3(
      @moonphys.identity_basis3(),
      @moonphys.vec3(2.0, 0.0, 0.0),
    ),
  )
  let manifold = @moonphys.collision_contact_manifold("m", [
    first, second, clear,
  ])
  assert_eq(manifold.contact_count, 2)
  assert_eq(manifold.status, "multi-contact")
  assert_true(nearly_equal(manifold.max_penetration_m, 0.25))
  assert_true(manifold.average_normal.x > 0.7)
  assert_true(manifold.average_normal.y > 0.7)
}

///|
test "collision manifold resolution separates and damps one body" {
  let contact = @moonphys.sphere_sphere_contact(
    @moonphys.collision_sphere("body", 0.5),
    @moonphys.transform3(
      @moonphys.identity_basis3(),
      @moonphys.vec3(0.0, 0.0, 0.0),
    ),
    @moonphys.collision_sphere("blocker", 0.5),
    @moonphys.transform3(
      @moonphys.identity_basis3(),
      @moonphys.vec3(0.75, 0.0, 0.0),
    ),
  )
  let manifold = @moonphys.collision_contact_manifold("m", [contact])
  let state = @moonphys.rigid_body_state(
    "body",
    4.0,
    0.5,
    @moonphys.vec3(0.0, 0.0, 0.0),
    @moonphys.vec3(2.0, 1.0, 0.0),
  )
  let resolution = @moonphys.resolve_collision_manifold(
    state,
    manifold,
    @moonphys.material_contact_model("test", 0.25, 0.1),
  )
  assert_eq(resolution.status, "single-contact-resolved")
  assert_true(nearly_equal(resolution.correction.x, -0.25))
  assert_true(nearly_equal(resolution.after.position.x, -0.25))
  assert_true(nearly_equal(resolution.normal_speed_mps, 2.0))
  assert_true(nearly_equal(resolution.tangent_speed_mps, 1.0))
  assert_true(nearly_equal(resolution.after.velocity.x, -0.2))
  assert_true(nearly_equal(resolution.after.velocity.y, 0.75))
  assert_true(resolution.average_normal == @moonphys.vec3(1.0, 0.0, 0.0))
  assert_true(nearly_equal(resolution.velocity_delta.x, -2.2))
  assert_true(nearly_equal(resolution.velocity_delta.y, -0.25))
  assert_true(nearly_equal(resolution.normal_impulse_ns, 8.8))
  assert_true(nearly_equal(resolution.friction_impulse_ns, 1.0))
}

///|
test "collision manifold resolution handles multi and clear contacts" {
  let first = @moonphys.sphere_sphere_contact(
    @moonphys.collision_sphere("body", 0.5),
    @moonphys.transform3(
      @moonphys.identity_basis3(),
      @moonphys.vec3(0.0, 0.0, 0.0),
    ),
    @moonphys.collision_sphere("front", 0.5),
    @moonphys.transform3(
      @moonphys.identity_basis3(),
      @moonphys.vec3(0.75, 0.0, 0.0),
    ),
  )
  let second = @moonphys.sphere_sphere_contact(
    @moonphys.collision_sphere("body", 0.5),
    @moonphys.transform3(
      @moonphys.identity_basis3(),
      @moonphys.vec3(0.0, 0.0, 0.0),
    ),
    @moonphys.collision_sphere("side", 0.5),
    @moonphys.transform3(
      @moonphys.identity_basis3(),
      @moonphys.vec3(0.0, 0.8, 0.0),
    ),
  )
  let state = @moonphys.rigid_body_state(
    "body",
    4.0,
    0.5,
    @moonphys.vec3(0.0, 0.0, 0.0),
    @moonphys.vec3(0.0, 0.0, 0.0),
  )
  let multi = @moonphys.resolve_collision_manifold(
    state,
    @moonphys.collision_contact_manifold("multi", [first, second]),
    @moonphys.lunar_regolith_contact_model(),
  )
  assert_eq(multi.status, "multi-contact-resolved")
  assert_true(multi.correction.x < 0.0)
  assert_true(multi.correction.y < 0.0)
  let clear = @moonphys.resolve_collision_manifold(
    state,
    @moonphys.collision_contact_manifold("clear", []),
    @moonphys.lunar_regolith_contact_model(),
  )
  assert_eq(clear.status, "no-contact")
  assert_true(clear.after == state)
  assert_true(clear.velocity_delta == @moonphys.vec3(0.0, 0.0, 0.0))
  assert_true(nearly_equal(clear.normal_impulse_ns, 0.0))
  assert_true(nearly_equal(clear.friction_impulse_ns, 0.0))
}
