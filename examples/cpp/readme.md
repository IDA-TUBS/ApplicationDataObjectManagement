# Simple CPP Example 

## Configure the library 
Then check the configuration of the ip addresses and topics in the json files in *ApplicationDataObjectManagement/examples/cpp/config*.
Currently, the library only supports the simple configuration of a single publisher and subscriber, as is needed in this example.
Please make sure, that your configured topic number in *topics.json* is also defined in the Topic struct in */ApplicationDataObjectManagement/include/adom/parameters.h*

The examples will be build in the build process of the library.

## Execution of simple CPP example
This example only serves to easily demonstrate the operation of the library. Since utilizing this library brings its main benefits when communicating large data objects costs many communication resources, e.g. distributed publisher/ subscriber nodes that cannot communicate via shared memory.

Open two terminals for both of the test applications.
Please start the subscribing application before the publishing application. To start the subscribing application execute the following command:
```
./subscriber_test_application_entity [DIRECTORY NAME]
```

For example:
```
./subscriber_test_application_entity DIRECTORY_02
```

To start the publishing application in the second terminal execute:
```
./publisher_test_application_entity 
```
Since only single-publisher systems are currently supported, the directory belonging to the publisher does not have to be specified.