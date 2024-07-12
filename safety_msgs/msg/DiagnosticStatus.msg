#This message is created to problems with the diagnostic_msgs at common_msgs in ROS melodic
# Possible levels of operations
byte OK=0
byte WARN=1
byte ERROR=2
byte STALE=3

byte level # level of operation enumerated above 
string name # a description of the test/component reporting
string message # a description of the status
string hardware_id # a hardware unique string
safety_msgs/KeyValue[] values # an array of values associated with the status