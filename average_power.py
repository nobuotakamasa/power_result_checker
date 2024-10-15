import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
import yaml
import sys
import signal
from functools import partial

yamlfile = sys.argv[1]

# YAMLファイルを読み込む
with open(yamlfile, "r") as file:
    params = yaml.safe_load(file)

sensor_connections = params["sensor_input"]["connection"]
names = params["sensor_input"]["name"]

# データを表示
print("topic power")
print(params["sensor_input"]["voltage"])
print(params["sensor_input"]["current"])
print(params["sensor_input"]["name"])
#print(sensor_connections)
#for i in range(len(sensor_connections)):
#    print(f"{names[i]} {sensor_connections[i]}")

print("topic cpu_inner")
print(params["cpu_inner"]["value"])
print(params["cpu_inner"]["array"])

print("diagnostics")
statusname = params["diagnostics"]["statusname"]
print(statusname)
node = None
diagnostics = {}
#diagnostics_count = {}
def print_diagnostics():
    #print(diagnostics)
    for hardware_id in diagnostics:
        print(hardware_id)
        sum = {}
        print("------------- average --------------")
        num = diagnostics[hardware_id][0]
        keys = diagnostics[hardware_id][1]
        for key in keys:
            average = keys[key] / num
            print(f"{key}, {average}")

def print_powers_sensor():
    sum = powers[1]
    num = powers[0]
    print("---------- power averages -----------")
    for i in range(len(sum)):
        average = sum[i] / num
        print(f"{names[i]}, {average}")

topic_values = {}
def print_powers_builtin1():
    for value_name in topic_values:
        average = topic_values[value_name][1] / topic_values[value_name][0]
        print(f"{value_name} {average}")

topic_arrays = {}
def print_powers_builtin2():
    for value_name in topic_arrays:
        #print(topic_arrays[value_name])
        array = topic_arrays[value_name][1]
        num = topic_arrays[value_name][0]
        for i in range(len(array)):
            array[i] /= num
        print(f"{value_name} {array}")

start_time = None
# Ctrl-Cが押されたときのハンドラ
def signal_handler(sig, frame):
    print("======   printing averaged results   ======")
    print("====== diagnostics contents ======")
    print_diagnostics()
    print("======     sensor inputs    ======")
    print_powers_sensor()
    print("======     built in 1       ======")
    print_powers_builtin1()
    print("======     built in 2       ======")
    print_powers_builtin2()
    print("============ unix time ===========")
    seconds, nanoseconds = start_time.seconds_nanoseconds()
    print(f"ros time start:{seconds}.{nanoseconds}")    
    current_time = node.get_clock().now()
    seconds, nanoseconds = current_time.seconds_nanoseconds()
    print(f"ros time end  :{seconds}.{nanoseconds}")    
    print("==================================")

    sys.exit(0)  # 正常終了

# SIGINT（Ctrl-C）のシグナルをキャッチするように設定
signal.signal(signal.SIGINT, signal_handler)

first = True
def init_timer():
    global first
    global start_time
    global node
    if first:
        first = False
        start_time = node.get_clock().now()
        seconds, nanoseconds = start_time.seconds_nanoseconds()
        print(f"ros time start:{seconds}.{nanoseconds}")    

def diagnostics_callback(msg):
    init_timer()
    for status in msg.status:
        if statusname in status.name:
            if diagnostics.get(status.hardware_id) == None:
                diagnostics[status.hardware_id] = [0, {}]
                for value in status.values:
                    diagnostics[status.hardware_id][1][value.key] = 0.0
            for value in status.values:
                diagnostics[status.hardware_id][1][value.key] += float(value.value)
            diagnostics[status.hardware_id][0] += 1

voltages = [0.0,0.0]
#[count, [1.2, 1.3, 1.4,....]]
powers = [0, [0.0] * len(sensor_connections)]
def callback_voltage(msg):
    init_timer()
    global voltages
    voltages = msg.data
    #print(msg.params)

def callback_current(msg):
    init_timer()
    #global currents
    global powers_count
    currents = msg.data
    power = []
    for i in range(len(currents)):
        #print(currents[i], voltages, type(sensor_connections[i]))
        power.append(currents[i] * voltages[sensor_connections[i]])
    powers[0] += 1
    for i in range(len(power)):
        powers[1][i] += power[i]

def callback_topic_array(name, msg):
    init_timer()
    print(name, msg.data)
    length = len(msg.data)
    if topic_arrays.get(name) == None:
        topic_arrays[name] = [0, [0.0] * length]
    #topic_arrays[name].append(msg.data)
    topic_arrays[name][0] += 1
    for i in range(length):
        topic_arrays[name][1][i] += msg.data[i]

def callback_topic_value(name, msg):
    init_timer()
    print(name, msg.data)
    if topic_values.get(name) == None:
        topic_values[name] = [0,0.0]
    topic_values[name][0] += 1
    topic_values[name][1] += msg.data

def main(args=None):
    rclpy.init(args=args)

    # Create a node instance
    global node
    node = Node('array_average')
    subscriptions = []
    #collecting dignostics
    subscriptions.append(node.create_subscription(
        DiagnosticArray, '/diagnostics', diagnostics_callback,10))

    #collecting voltages
    subscriptions.append(node.create_subscription(
        Float32MultiArray,
        params["sensor_input"]["voltage"],
        callback_voltage, 10))

    #collecting currents
    subscriptions.append(node.create_subscription(
        Float32MultiArray,
        params["sensor_input"]["current"],
        callback_current, 100))

    topic_array = params["cpu_inner"]["array"]
    for name in topic_array:
        subscriptions.append(node.create_subscription(
            Float32MultiArray,
            name,
            lambda msg, name=name: callback_topic_array(name,msg), 10))

    topic_value = params["cpu_inner"]["value"]
    for name in topic_value:
        subscriptions.append(node.create_subscription(
            Float32,
            name,
            lambda msg, name=name: callback_topic_value(name,msg), 10))

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        pass
        #node.destroy_node()
        #rclpy.shutdown()



if __name__ == '__main__':
    main()
