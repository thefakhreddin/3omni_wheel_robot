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

//  odom.pose.pose.position.x = x_pos;                       // odom section
//  odom.pose.pose.position.y = y_pos;
//  odom.pose.pose.orientation.w = cos(alpha / 2);
//  odom.pose.pose.orientation.z = sin(alpha / 2);
//
//  odom.child_frame_id = base_link;
//  odom.twist.twist.linear.x = vx_ac;
//  odom.twist.twist.linear.y = vy_ac;
//  odom.twist.twist.angular.z = w_ac;
//  odom_pub.publish(&odom);
}
