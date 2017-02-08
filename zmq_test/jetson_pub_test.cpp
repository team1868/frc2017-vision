#include "zhelpers.hpp"
#include <string>
#include <chrono>
#include <thread>

using namespace zmq;
using namespace std;

int main () {
    //  Prepare our context and publisher
    context_t context(1);
    socket_t publisher(context, ZMQ_PUB);

    // cout << publisher.setsockopt(ZMQ_SNDHWM, 1) << endl;
    // cout << publisher.setsockopt(ZMQ_SNDHWM, 1) << endl;
    //int rc = zmq_setsockopt (publisher, ZMQ_SNDHWM, 1);
    //assert (rc == 0);
    publisher.bind("tcp://*:5563");

    for (int i = 0; i < 2000000; i++) {
        //  Write two messages, each with an envelope and content
        s_sendmore (publisher, "A");
        s_send (publisher, "We don't want to see this");
        s_sendmore (publisher, "B");
	   string pub_string = to_string(i);
        s_send (publisher, pub_string);
	//s_send (publisher, i);
	this_thread::sleep_for(chrono::milliseconds(1));
    }
    return 0;
}
