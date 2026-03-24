import rclpy
from rclpy.node import Node

# import rospy
from std_msgs.msg import Float32, Float32MultiArray


class AnalogVoltageInterpreterNode(Node):

    def __init__(self):
        super().__init__('analog_voltage_interpreter')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'ai1616llpe/voltage',
            self.callback,
            10)
        self.subscription  # prevent unused variable warning

        # indexes_string = ''
        # indexes_string = self.declare_parameter('~ai_indexes', '1,2').value

        # self.get_logger().info("ai_indexes_string = %s"%(indexes_string))

        # # indexes are splitted by camma
        # index_string_list = indexes_string.split(',')
        # self.index_list = [int(i) for i in index_string_list]

        self.declare_parameters(
            namespace='',
            parameters=[
                ('~ai_indexes', 1),
            ]
        )

        ai_index = self.get_parameter('~ai_indexes').value
        self.ai_index_list = [int(i) for i in range(ai_index)]

        self.publisher_ = self.create_publisher(
            Float32MultiArray, '/pressure', 10)

    def callback(self, msg):
        outputs = Float32MultiArray()

        # for i, voltage in enumerate(msg.data):
        # if(i == 0):
        #     outputs.data.append(self.get_converted_value(voltage, i))
        # if(i == 1):
        #     outputs.data.append(self.get_force_value(voltage, i))
        for i in range(len(self.ai_index_list)):
            outputs.data.append(self.get_converted_value(msg.data[i], i))

        # self.pub.publish(outputs)
        self.publisher_.publish(outputs)

        # self.get_logger().info('outputs.data[0]: "%f"' % outputs.data[0])

    def get_converted_value(self, voltage, i):
        # return voltage       # return raw voltage.

        # pressure value [kPa]
        zero_pressure_voltage = 1.0  # 1.05   # 1.0
        maximum_pressure_voltage = 5.0

        maximum_pressure = 1000.0  # 900.0   # 1000.0
        # if i == 2:
        #     maximum_pressure = 101.0 #
        # else:
        #     maximum_pressure = 1000.0 # 900.0   # 1000.0

        pressure = (voltage - zero_pressure_voltage) * maximum_pressure / \
            (maximum_pressure_voltage - zero_pressure_voltage)

        return pressure

    def get_force_value(self, voltage, i):
        # return voltage #return raw voltage

        # force vakue[N]
        zero_force_voltage = 0.0
        maximum_force_voltage = 1.0
        maximum_force = 20.0

        force = (voltage - zero_force_voltage) * maximum_force / \
            (maximum_force_voltage - zero_force_voltage)

        return force


def main(args=None):
    rclpy.init(args=args)

    analog_voltage_interpreter = AnalogVoltageInterpreterNode()

    rclpy.spin(analog_voltage_interpreter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    analog_voltage_interpreter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


# class AnalogVoltageInterpreterNode:
# # cdef class AnalogVoltageInterpreterNode:

#     # cdef list index_list
#     # cdef object pub, data_received_sub, pub_each
#     # cdef str node_name

#     def __init__(self):

#         # cdef list index_list, index_string_list
#         # cdef str i, node_name

#         # get indexes of data interpreted
#         cdef char* indexes_string = ''
#         if(rospy.has_param('~indexes')):
#             temp_string = rospy.get_param('~indexes')
#             indexes_string = temp_string
#         else:
#             rospy.logerr('indexes are not registered: ' + rospy.get_name())

#         rospy.loginfo("indexes_string = %s"%(indexes_string))

#         # indexes are splitted by camma
#         index_string_list = indexes_string.split(',')
#         # rospy.loginfo("index_string_list[0] = %s, index_string_list[1] = %s"%(index_string_list[0], index_string_list[1]))

#         # self.a = 3
#         # rospy.loginfo("%d"%self.a)

#         # self.index_list = map(int, index_string_list)
#         self.index_list = [int(i) for i in index_string_list]
#         # rospy.loginfo("index_list[0] = %s, index_list[1] = %s"%(self.index_list[0], self.index_list[1]))


#         # loop_rate = None
#         # if(rospy.has_param('~loop_rate')):
#         #     loop_rate = rospy.get_param('~loop_rate')
#         # else:
#         #     rospy.logerr('control_parameters/update_rate is not registered')
#         #     return
#         # self.loop_rate = rospy.Rate(loop_rate)
#         # rospy.loginfo("[%s] loop_rate: %s", self.__class__.__name__, loop_rate)

#         published_topic_name = rospy.get_name() + '/value'
#         # subscribed_topic_name = ('~analog_voltage')
#         # test = rospy.Publisher(
#         #     published_topic_name, Float32MultiArray, queue_size=10)
#         # rospy.loginfo("self.pub type = %s"%((test)))
#         self.pub = rospy.Publisher(
#             published_topic_name, Float32MultiArray, queue_size=10)

#         self.node_name = rospy.get_name()

#     cdef run(self):

#         self.data_received_sub = rospy.Subscriber(
#             '~analog_voltage', Float32MultiArray, self.data_received_callback)

#         # rospy.loginfo("data_received_callback type = %s"%((self.data_received_callback)))

#         # self.loop_rate.sleep()

# #        # if function run() is executed not in main module,
# #        # rospy.spin() in this function is not executed
# #        if(__name__ == '__main__'):
# #            rospy.spin()
# #        else:
# #            pass

#         rospy.spin()

#     cpdef data_received_callback(self, msg):
#         outputs = Float32MultiArray()
#         # data whoose index is contained in index_list is collected,
#         # and is converted with get_converted_value()

#         cdef int i
#         cdef float voltage
#         cdef tuple msg_data = msg.data
#         cdef char* each_name
#         cdef list index_list

#         # rospy.loginfo("data_type of msg = %s", (msg.data))

#         for i, voltage in enumerate(msg_data):
#             if(i in self.index_list):
#                 outputs.data.append(self.get_converted_value(voltage))

#         cdef list outputs_data = outputs.data

#         # # publish each data (heavy process if the data number is large.)
#         # for i in range(len(outputs_data)):
#         #     temp_name = self.node_name + '/value' + '_' + str(i)
#         #     each_name = temp_name
#         #     self.pub_each = rospy.Publisher(each_name, Float32, queue_size=10)
#         #     self.pub_each.publish(outputs.data[i])

#         self.pub.publish(outputs)
#         # self.loop_rate.sleep()

#         rospy.loginfo("%s.data[0] = %s"%(self.node_name, outputs.data[0]))

#     cdef get_converted_value(self, float voltage):
#         return voltage

# cpdef main():
#     # rospy.init_node('~node_name')
#     rclpy.init(args=sys.argv)
#     node = rclpy.create_node('~node_name')

#     node = AnalogVoltageInterpreterNode()
#     node.run()

# if __name__ == '__main__':
#     main()
