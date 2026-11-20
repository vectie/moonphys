///|
test "joint servo output applies pd torque and saturation" {
  let state = @moonphys.joint_motor_state("hip", 0.0, 0.0)
  let command = @moonphys.joint_motor_command("hip", 1.0, 0.0, 0.0)
  let config = @moonphys.joint_servo_config("hip", 8.0, 1.0)
  let limit = @moonphys.joint_limit("hip", -2.0, 2.0, 4.0, 3.0)
  let servo = @moonphys.joint_servo_output(state, command, config, limit)
  assert_true(nearly_equal(servo.position_error_rad, 1.0))
  assert_true(nearly_equal(servo.commanded_torque_nm, 3.0))
  assert_true(servo.saturated)
  assert_eq(servo.status, "torque-saturated")
}

///|
test "joint motor integration applies torque semi implicitly" {
  let step = @moonphys.integrate_joint_motor(
    @moonphys.joint_motor_state("knee", 0.0, 0.0),
    @moonphys.joint_motor_command("knee", 1.0, 0.0, 0.0),
    @moonphys.joint_servo_config("knee", 2.0, 0.0),
    @moonphys.joint_limit("knee", -2.0, 2.0, 4.0, 10.0),
    2.0,
    0.5,
  )
  assert_eq(step.status, "torque-commanded")
  assert_true(nearly_equal(step.after.velocity_rad_s, 0.5))
  assert_true(nearly_equal(step.after.position_rad, 0.25))
  assert_true(nearly_equal(step.work_j, 0.5))
  assert_true(nearly_equal(step.absolute_work_j, 0.5))
  assert_true(nearly_equal(step.average_mechanical_power_w, 1.0))
}

///|
test "joint motor integration respects velocity and position limits" {
  let step = @moonphys.integrate_joint_motor(
    @moonphys.joint_motor_state("ankle", 0.9, 0.0),
    @moonphys.joint_motor_command("ankle", 5.0, 0.0, 0.0),
    @moonphys.joint_servo_config("ankle", 100.0, 0.0),
    @moonphys.joint_limit("ankle", -1.0, 1.0, 0.5, 100.0),
    1.0,
    1.0,
  )
  assert_eq(step.status, "position-limit")
  assert_true(nearly_equal(step.after.position_rad, 1.0))
  assert_true(nearly_equal(step.after.velocity_rad_s, 0.0))
  assert_true(nearly_equal(step.work_j, 10.0))
  assert_true(nearly_equal(step.average_mechanical_power_w, 10.0))
}

///|
test "joint velocity limited command shapes unreachable targets" {
  let state = @moonphys.joint_motor_state("ankle", 0.0, 0.0)
  let limit = @moonphys.joint_limit("ankle", -1.0, 1.0, 0.5, 100.0)
  let shaped = @moonphys.joint_velocity_limited_command(
    state, 0.4, 4.0, 0.0, limit, 0.2,
  )
  assert_eq(shaped.status, "joint-command-velocity-limited")
  assert_true(shaped.velocity_limited)
  assert_true(!shaped.position_limited)
  assert_true(nearly_equal(shaped.desired_delta_rad, 0.4))
  assert_true(nearly_equal(shaped.applied_delta_rad, 0.1))
  assert_true(nearly_equal(shaped.command.target_position_rad, 0.1))
  assert_true(nearly_equal(shaped.command.target_velocity_rad_s, 0.5))
  let frame = @moonphys.integrate_joint_motor_frame(
    "velocity-limited-frame",
    [state],
    [shaped.command],
    [@moonphys.joint_servo_config("ankle", 2.0, 0.0)],
    [limit],
    [1.0],
    0.2,
  )
  assert_eq(frame.limit_review_count, 0)
  assert_true(frame.steps[0].velocity_within_limits)
}

///|
test "joint motor frame integrates and aggregates multiple joints" {
  let frame = @moonphys.integrate_joint_motor_frame(
    "frame-0",
    [
      @moonphys.joint_motor_state("hip", 0.0, 0.0),
      @moonphys.joint_motor_state("knee", 0.0, 0.0),
    ],
    [
      @moonphys.joint_motor_command("hip", 1.0, 0.0, 0.0),
      @moonphys.joint_motor_command("knee", 0.0, 5.0, 0.0),
    ],
    [
      @moonphys.joint_servo_config("hip", 2.0, 0.0),
      @moonphys.joint_servo_config("knee", 0.0, 1.0),
    ],
    [
      @moonphys.joint_limit("hip", -2.0, 2.0, 4.0, 10.0),
      @moonphys.joint_limit("knee", -2.0, 2.0, 3.0, 10.0),
    ],
    [1.0, 1.0],
    0.5,
  )
  assert_eq(frame.status, "joint-frame-limit-review")
  assert_eq(frame.joint_count, 2)
  assert_eq(frame.saturated_joint_count, 0)
  assert_eq(frame.limit_review_count, 1)
  assert_true(frame.steps[0].position_within_limits)
  assert_true(!frame.steps[1].velocity_within_limits)
  assert_true(nearly_equal(frame.max_abs_velocity_rad_s, 5.0))
  assert_true(nearly_equal(frame.max_abs_commanded_torque_nm, 5.0))
  assert_true(nearly_equal(frame.total_absolute_work_j, 7.25))
}

///|
test "transform translates local point" {
  let transform = @moonphys.transform3(
    @moonphys.identity_basis3(),
    @moonphys.vec3(1.0, 2.0, 3.0),
  )
  assert_true(
    transform.transform_point(@moonphys.vec3(4.0, 5.0, 6.0)) ==
    @moonphys.vec3(5.0, 7.0, 9.0),
  )
}

///|
test "axis angle basis rotates around z" {
  let basis = @moonphys.axis_angle_basis3(
    @moonphys.vec3(0.0, 0.0, 1.0),
    3.141592653589793 / 2.0,
  )
  let rotated = basis.mul_vec3(@moonphys.vec3(1.0, 0.0, 0.0))
  assert_true(rotated.x < 0.000001)
  assert_true(rotated.y > 0.999999)
}

///|
test "transform composition applies parent translation and local rotation" {
  let parent = @moonphys.transform3(
    @moonphys.identity_basis3(),
    @moonphys.vec3(1.0, 0.0, 0.0),
  )
  let child = @moonphys.transform3(
    @moonphys.axis_angle_basis3(
      @moonphys.vec3(0.0, 0.0, 1.0),
      3.141592653589793 / 2.0,
    ),
    @moonphys.vec3(0.0, 2.0, 0.0),
  )
  let composed = parent.compose(child)
  let point = composed.transform_point(@moonphys.vec3(1.0, 0.0, 0.0))
  assert_true(point.x < 1.000001 && point.x > 0.999999)
  assert_true(point.y > 2.999999)
}

///|
test "articulated pose tree resolves parent first chain transforms" {
  let tree = @moonphys.articulated_pose_tree(
    @moonphys.transform3(
      @moonphys.identity_basis3(),
      @moonphys.vec3(0.0, 0.0, 0.0),
    ),
    [
      @moonphys.articulated_link_spec(
        "base",
        "",
        "",
        @moonphys.vec3(0.0, 0.0, 0.0),
        @moonphys.vec3(0.0, 0.0, 1.0),
      ),
      @moonphys.articulated_link_spec(
        "shoulder",
        "base",
        "shoulder_yaw",
        @moonphys.vec3(1.0, 0.0, 0.0),
        @moonphys.vec3(0.0, 0.0, 1.0),
      ),
      @moonphys.articulated_link_spec(
        "elbow",
        "shoulder",
        "elbow_fixed",
        @moonphys.vec3(1.0, 0.0, 0.0),
        @moonphys.vec3(0.0, 0.0, 1.0),
      ),
    ],
    [
      @moonphys.articulated_joint_position(
        "shoulder_yaw",
        3.141592653589793 / 2.0,
      ),
    ],
  )
  let shoulder = tree.link_transform("shoulder").translation
  let elbow = tree.link_transform("elbow").translation
  assert_eq(tree.status, "articulated-poses")
  assert_eq(tree.link_count, 3)
  assert_true(shoulder == @moonphys.vec3(1.0, 0.0, 0.0))
  assert_true(nearly_equal(elbow.x, 1.0))
  assert_true(nearly_equal(elbow.y, 1.0))
}

///|
test "articulated pose tree reports missing parent" {
  let tree = @moonphys.articulated_pose_tree(
    @moonphys.transform3(
      @moonphys.identity_basis3(),
      @moonphys.vec3(2.0, 0.0, 0.0),
    ),
    [
      @moonphys.articulated_link_spec(
        "forearm",
        "missing-upper-arm",
        "elbow",
        @moonphys.vec3(1.0, 0.0, 0.0),
        @moonphys.vec3(0.0, 0.0, 1.0),
      ),
    ],
    [],
  )
  assert_eq(tree.status, "articulated-poses-missing-parent-review")
  assert_eq(tree.missing_parent_count, 1)
  assert_eq(tree.links[0].status, "missing-parent")
  assert_true(
    tree.links[0].world_transform.translation == @moonphys.vec3(3.0, 0.0, 0.0),
  )
}

///|
test "support assessment reports static margin" {
  let assessment = @moonphys.support_assessment(@moonphys.vec3(0.0, 0.0, 1.0), [
    @moonphys.support_footprint("left", @moonphys.vec3(0.0, 0.0, 0.0), 0.2, 0.1),
  ])
  assert_true(assessment.stable)
  assert_eq(assessment.support_count, 1)
  assert_eq(assessment.support_polygon_vertex_count, 4)
  assert_true(nearly_equal(assessment.planar_margin_m, 0.1))
  assert_eq(assessment.status, "inside-support")
}

///|
test "support assessment flags projection outside support" {
  let assessment = @moonphys.support_assessment(@moonphys.vec3(0.0, 0.0, 1.0), [
    @moonphys.support_footprint(
      "left",
      @moonphys.vec3(0.0, 0.2, 0.0),
      0.2,
      0.05,
    ),
  ])
  assert_true(!assessment.stable)
  assert_true(assessment.planar_margin_m < 0.0)
  assert_eq(assessment.status, "outside-support")
}

///|
test "support recovery correction targets nearest support boundary" {
  let correction = @moonphys.support_recovery_correction(
    @moonphys.vec3(0.0, 0.0, 1.0),
    [
      @moonphys.support_footprint(
        "left",
        @moonphys.vec3(0.0, 0.2, 0.0),
        0.2,
        0.05,
      ),
    ],
  )
  assert_true(!correction.stable_without_shift)
  assert_true(correction.recoverable)
  assert_eq(correction.status, "support-recovery-shift-required")
  assert_true(nearly_equal(correction.target_point.x, 0.0))
  assert_true(nearly_equal(correction.target_point.y, 0.15))
  assert_true(nearly_equal(correction.required_shift.y, 0.15))
  assert_true(nearly_equal(correction.required_shift_m, 0.15))
  assert_true(correction.recovered_margin_m >= 0.0)
}

///|
test "support recovery correction is idle for stable support" {
  let correction = @moonphys.support_recovery_correction(
    @moonphys.vec3(0.0, 0.0, 1.0),
    [
      @moonphys.support_footprint(
        "left",
        @moonphys.vec3(0.0, 0.0, 0.0),
        0.2,
        0.1,
      ),
    ],
  )
  assert_true(correction.stable_without_shift)
  assert_true(correction.recoverable)
  assert_eq(correction.status, "support-recovery-not-needed")
  assert_true(nearly_equal(correction.required_shift_m, 0.0))
}

///|
test "support assessment uses convex support polygon" {
  let footprints = [
    @moonphys.support_footprint(
      "rear-left",
      @moonphys.vec3(0.0, 0.0, 0.0),
      0.1,
      0.1,
    ),
    @moonphys.support_footprint(
      "front-right",
      @moonphys.vec3(1.0, 1.0, 0.0),
      0.1,
      0.1,
    ),
  ]
  let inside = @moonphys.support_assessment(
    @moonphys.vec3(0.5, 0.5, 1.0),
    footprints,
  )
  assert_true(inside.stable)
  assert_eq(inside.support_count, 2)
  assert_true(inside.support_polygon_vertex_count > 4)
  assert_true(inside.planar_margin_m > 0.0)
  let outside_hull = @moonphys.support_assessment(
    @moonphys.vec3(0.9, 0.1, 1.0),
    footprints,
  )
  assert_true(!outside_hull.stable)
  assert_true(outside_hull.margin_x_m > 0.0)
  assert_true(outside_hull.margin_y_m > 0.0)
  assert_true(outside_hull.planar_margin_m < 0.0)
  assert_eq(outside_hull.status, "outside-support")
}

///|
test "support assessment handles no active support" {
  let assessment = @moonphys.support_assessment(@moonphys.vec3(0.0, 0.0, 1.0), [
    @moonphys.support_footprint(
      "left",
      @moonphys.vec3(0.0, 0.0, 0.0),
      0.2,
      0.1,
      active=false,
    ),
  ])
  assert_true(!assessment.stable)
  assert_eq(assessment.support_count, 0)
  assert_eq(assessment.status, "no-support")
}

///|
test "support load assessment splits two-foot normal force" {
  let assessment = @moonphys.support_load_assessment(
    @moonphys.vec3(0.5, 0.0, 1.0),
    [
      @moonphys.support_footprint(
        "left",
        @moonphys.vec3(0.0, 0.0, 0.0),
        0.2,
        0.1,
      ),
      @moonphys.support_footprint(
        "right",
        @moonphys.vec3(1.0, 0.0, 0.0),
        0.2,
        0.1,
      ),
    ],
    120.0,
  )
  assert_eq(assessment.load_count, 2)
  assert_true(assessment.balanced)
  assert_eq(assessment.status, "center-load-balanced")
  assert_true(nearly_equal(assessment.loads[0].load_fraction, 0.5))
  assert_true(nearly_equal(assessment.loads[1].load_fraction, 0.5))
  assert_true(nearly_equal(assessment.loads[0].normal_force_n, 60.0))
  assert_true(nearly_equal(assessment.loads[1].normal_force_n, 60.0))
  assert_true(nearly_equal(assessment.center_of_pressure_error_m, 0.0))
}

///|
test "support load assessment reports off-segment center-load error" {
  let assessment = @moonphys.support_load_assessment(
    @moonphys.vec3(0.5, 0.2, 1.0),
    [
      @moonphys.support_footprint(
        "left",
        @moonphys.vec3(0.0, 0.0, 0.0),
        0.2,
        0.1,
      ),
      @moonphys.support_footprint(
        "right",
        @moonphys.vec3(1.0, 0.0, 0.0),
        0.2,
        0.1,
      ),
    ],
    120.0,
  )
  assert_true(!assessment.balanced)
  assert_eq(assessment.status, "outside-support-load-review")
  assert_true(nearly_equal(assessment.center_of_pressure_error_m, 0.2))
}

///|
test "support load assessment accepts area-supported center" {
  let assessment = @moonphys.support_load_assessment(
    @moonphys.vec3(0.0, -0.04, 1.0),
    [
      @moonphys.support_footprint(
        "left",
        @moonphys.vec3(0.08, 0.08, 0.0),
        0.11,
        0.045,
      ),
      @moonphys.support_footprint(
        "right",
        @moonphys.vec3(-0.04, -0.08, 0.0),
        0.11,
        0.045,
      ),
    ],
    120.0,
  )
  assert_true(assessment.balanced)
  assert_eq(assessment.status, "footprint-area-load")
  assert_true(assessment.center_of_pressure_error_m > 0.0)
}

///|
test "support load assessment uses triangle barycentric fractions" {
  let assessment = @moonphys.support_load_assessment(
    @moonphys.vec3(0.25, 0.25, 1.0),
    [
      @moonphys.support_footprint(
        "a",
        @moonphys.vec3(0.0, 0.0, 0.0),
        0.05,
        0.05,
      ),
      @moonphys.support_footprint(
        "b",
        @moonphys.vec3(1.0, 0.0, 0.0),
        0.05,
        0.05,
      ),
      @moonphys.support_footprint(
        "c",
        @moonphys.vec3(0.0, 1.0, 0.0),
        0.05,
        0.05,
      ),
    ],
    100.0,
  )
  assert_true(assessment.balanced)
  assert_eq(assessment.status, "center-load-balanced")
  assert_true(nearly_equal(assessment.loads[0].load_fraction, 0.5))
  assert_true(nearly_equal(assessment.loads[1].load_fraction, 0.25))
  assert_true(nearly_equal(assessment.loads[2].load_fraction, 0.25))
  assert_true(nearly_equal(assessment.center_of_pressure_error_m, 0.0))
}

///|
test "support load assessment handles no active support" {
  let assessment = @moonphys.support_load_assessment(
    @moonphys.vec3(0.0, 0.0, 1.0),
    [
      @moonphys.support_footprint(
        "left",
        @moonphys.vec3(0.0, 0.0, 0.0),
        0.2,
        0.1,
        active=false,
      ),
    ],
    120.0,
  )
  assert_true(!assessment.balanced)
  assert_eq(assessment.load_count, 0)
  assert_eq(assessment.status, "no-support-load")
}

///|
test "traction assessment reports friction margin and slip" {
  let material = @moonphys.material_contact_model("test", 0.5, 0.0)
  let ok = @moonphys.traction_assessment("foot", material, 100.0, 20.0)
  assert_eq(ok.status, "traction-ok")
  assert_true(nearly_equal(ok.friction_limit_n, 50.0))
  assert_true(nearly_equal(ok.utilization, 0.4))
  assert_true(nearly_equal(ok.margin_n, 30.0))
  assert_true(!ok.slipping)
  let low = @moonphys.traction_assessment("foot", material, 100.0, 45.0)
  assert_eq(low.status, "friction-margin-low")
  assert_true(nearly_equal(low.utilization, 0.9))
  let slip = @moonphys.traction_assessment("foot", material, 100.0, 60.0)
  assert_eq(slip.status, "friction-limit-exceeded")
  assert_true(slip.slipping)
  assert_true(nearly_equal(slip.margin_n, 0.0 - 10.0))
  let no_normal = @moonphys.traction_assessment("foot", material, 0.0, 1.0)
  assert_eq(no_normal.status, "no-normal-force-slip")
  assert_true(no_normal.slipping)
}

///|
test "surface traction assessment projects force onto flat normal" {
  let material = @moonphys.material_contact_model("test", 0.5, 0.0)
  let assessment = @moonphys.surface_traction_assessment(
    "foot",
    material,
    @moonphys.vec3(3.0, 0.0, 10.0),
    @moonphys.vec3(0.0, 0.0, 1.0),
  )
  assert_true(nearly_equal(assessment.normal_force_n, 10.0))
  assert_true(nearly_equal(assessment.tangential_force_n, 3.0))
  assert_eq(assessment.status, "traction-ok")
}

///|
test "surface traction assessment exposes slope tangential demand" {
  let material = @moonphys.material_contact_model("test", 0.5, 0.0)
  let normal = @moonphys.vec3(0.0, 1.0, 1.0)
  let assessment = @moonphys.surface_traction_assessment(
    "foot",
    material,
    @moonphys.vec3(0.0, 0.0, 10.0),
    normal,
  )
  let expected = 50.0.sqrt()
  assert_true(nearly_equal(assessment.normal_force_n, expected))
  assert_true(nearly_equal(assessment.tangential_force_n, expected))
  assert_eq(assessment.status, "friction-limit-exceeded")
}

///|
test "capture point assessment keeps slow COM inside support" {
  let assessment = @moonphys.capture_point_assessment(
    @moonphys.lunar_environment(),
    @moonphys.vec3(0.0, 0.0, 0.7),
    @moonphys.vec3(0.02, 0.0, 0.0),
    [
      @moonphys.support_footprint(
        "left",
        @moonphys.vec3(0.0, 0.0, 0.0),
        0.2,
        0.1,
      ),
    ],
    nominal_com_height_m=0.7,
  )
  assert_true(assessment.stable)
  assert_eq(assessment.status, "capture-point-inside-support")
  assert_true(assessment.omega_rad_s > 0.0)
  assert_true(assessment.capture_point.x > assessment.com_position.x)
}

///|
test "capture point assessment flags fast COM outside support" {
  let assessment = @moonphys.capture_point_assessment(
    @moonphys.lunar_environment(),
    @moonphys.vec3(0.0, 0.0, 0.7),
    @moonphys.vec3(0.6, 0.0, 0.0),
    [
      @moonphys.support_footprint(
        "left",
        @moonphys.vec3(0.0, 0.0, 0.0),
        0.2,
        0.1,
      ),
    ],
    nominal_com_height_m=0.7,
  )
  assert_true(!assessment.stable)
  assert_eq(assessment.status, "capture-point-outside-support")
  assert_true(assessment.capture_margin_m < 0.0)
}
