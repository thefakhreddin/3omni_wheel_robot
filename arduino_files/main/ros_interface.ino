void compute_tf_and_odom() {                                // computing and sending trasform over tf & odom over ros

  t.header.frame_id = home_link;                            // tf section
  t.header.stamp = nh.now();
  t.child_frame_id = base_link;
  t.transform.translation.x = x_pos;
  t.transform.translation.y = y_pos;
  t.transform.translation.z = 0;
  t.transform.rotation.w = cos(alpha / 2);
  t.transform.rotation.z = sin(alpha / 2);
  broadcaster.sendTransform(t);

  base_pos.x = x_pos;                                       // odom section
  base_pos.y = y_pos;
  base_pos.theta = alpha;

  base_vel.x = vx_ac;
  base_vel.y = vy_ac;
  base_vel.theta = w_ac;

  odom_pos.publish(&base_pos);
  odom_vel.publish(&base_vel);
}
