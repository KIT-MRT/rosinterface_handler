# Topic handling via a topic sever

Specifying topics via launchfiles an yaml files is very error-prone, especially in large configurations and often yields unconnected nodes which are hard to find and debug.
The *topic server* approach solves this by using a centralized server node where all nodes will ask for the topics they want to use. This has many **advantages**:
- The server can operate based on a **central configuration file** (or some other intelligent approach), instead of distributed launchfiles
- **Improved introspection** at runtime using the server since the server knows all topics
- **Improved debugging and identification of unconnected nodes** or message type mismatches since the sever knows all already advertised topics

The `rosinterface_handler` provides facilities to make this work, but this comes at a few **disadvantages**:
- You have to implement the server. The rosinterface_handler just offers the [`GetTopic`](../srv/GetTopic.srv) service that you have to implement on the server side
- Ideally all your nodes need to make use of the `add_subscriber` and `add_publisher` interface generator functions so that all nodes get ther topics assigned from `rosinterface_handler`

## How to make this work
No change is required on the node/nodelet side if all your C++/Python nodes use `rosinterface_handler`.
The `rosinterface_handler` backend will direct all topic requests to the server if the **ros parameter `/topic_server`** is set to the topic server node.

Have a look at the [dummy_topic_server](../test/python/dummy_topic_server.py) implementation which is launched by the [dummy_topic_server.launch](../test/launch/dummy_topic_server.launch) for an example.
Note that we are using the same test nodes with a running and a non-running topic server. As said, the nodes need no change.
