#include <iostream>
#include <fstream>
#include <string>
#include <vector>
// c++11 sleep
#include <thread>
#include <chrono>

#include "Net.h"

#define SHOW_ACKS

using namespace std;
using namespace net;

const unsigned short ServerPort = 30000;
const unsigned short ClientPort = 30001;
const int ProtocolId = 0x11112222;
const float DeltaTime = 0.001f;
const chrono::milliseconds DeltaTimeInMs(1);
const float TimeOut = 0.1f;
const int PacketSize = 256;

	// const int ServerPort = 30000;
	// const int ClientPort = 30001;
	// const int ProtocolId = 0x11112222;
	// const float DeltaTime = 0.001f;
	// const float TimeOut = 0.1f;
	// const uint32_t PacketCount = 100;

class FlowControl
{
public:
	
	FlowControl()
	{
		printf( "flow control initialized\n" );
		Reset();
	}
	
	void Reset()
	{
		mode = Bad;
		penalty_time = 4.0f;
		good_conditions_time = 0.0f;
		penalty_reduction_accumulator = 0.0f;
	}
	
	void Update( float deltaTime, float rtt )
	{
		const float RTT_Threshold = 250.0f;

		if ( mode == Good )
		{
			if ( rtt > RTT_Threshold )
			{
				printf( "*** dropping to bad mode ***\n" );
				mode = Bad;
				if ( good_conditions_time < 10.0f && penalty_time < 60.0f )
				{
					penalty_time *= 2.0f;
					if ( penalty_time > 60.0f )
						penalty_time = 60.0f;
					printf( "penalty time increased to %.1f\n", penalty_time );
				}
				good_conditions_time = 0.0f;
				penalty_reduction_accumulator = 0.0f;
				return;
			}
			
			good_conditions_time += deltaTime;
			penalty_reduction_accumulator += deltaTime;
			
			if ( penalty_reduction_accumulator > 10.0f && penalty_time > 1.0f )
			{
				penalty_time /= 2.0f;
				if ( penalty_time < 1.0f )
					penalty_time = 1.0f;
				printf( "penalty time reduced to %.1f\n", penalty_time );
				penalty_reduction_accumulator = 0.0f;
			}
		}
		
		if ( mode == Bad )
		{
			if ( rtt <= RTT_Threshold )
				good_conditions_time += deltaTime;
			else
				good_conditions_time = 0.0f;
				
			if ( good_conditions_time > penalty_time )
			{
				printf( "*** upgrading to good mode ***\n" );
				good_conditions_time = 0.0f;
				penalty_reduction_accumulator = 0.0f;
				mode = Good;
				return;
			}
		}
	}
	
	float GetSendRate()
	{
		return mode == Good ? 30.0f : 10.0f;
	}
	
private:

	enum Mode
	{
		Good,
		Bad
	};

	Mode mode;
	float penalty_time;
	float good_conditions_time;
	float penalty_reduction_accumulator;
};

// ----------------------------------------------

int main( int argc, char * argv[] )
{
	// parse command line

	enum Mode
	{
		Client,
		Server
	};

	Mode mode = Server;
	Address address;

	if ( argc >= 2 )
	{
		uchar_t a,b,c,d;
		if ( sscanf( argv[1], "%s.%s.%s.%s", &a, &b, &c, &d ) )
		{
			mode = Client;
			address = Address(a,b,c,d,ServerPort);
		}
	}

	// initialize

	if ( !InitializeSockets() )
	{
		printf( "failed to initialize sockets\n" );
		return 1;
	}

	ReliableConnection connection( ProtocolId, TimeOut );
	
	const unsigned short port = mode == Server ? ServerPort : ClientPort;
	
	if ( !connection.Start( port ) )
	{
		printf( "could not start connection on port %d\n", port );
		return 1;
	}
	
	if ( mode == Client )
		connection.Connect( address );
	else
		connection.Listen();
	
	bool connected = false;
	float sendAccumulator = 0.0f;
	float statsAccumulator = 0.0f;
	
	FlowControl flowControl;
	
	while ( true )
	{
		// update flow control
		
		if ( connection.IsConnected() )
			flowControl.Update( DeltaTime, connection.GetReliabilitySystem().GetRoundTripTime() * 1000.0f );
		
		const float sendRate = flowControl.GetSendRate();

		// detect changes in connection state

		if ( mode == Server && connected && !connection.IsConnected() )
		{
			flowControl.Reset();
			printf( "reset flow control\n" );
			connected = false;
		}

		if ( !connected && connection.IsConnected() )
		{
			printf( "client connected to server\n" );
			connected = true;
		}
		
		if ( !connected && connection.ConnectFailed() )
		{
			printf( "connection failed\n" );
			break;
		}
		
		// send and receive packets
		
		sendAccumulator += DeltaTime;
		
		while ( sendAccumulator > 1.0f / sendRate )
		{
			uchar_t packet[PacketSize];
			memset( packet, 0, sizeof( packet ) );
			connection.SendPacket( packet, sizeof( packet ) );
			sendAccumulator -= 1.0f / sendRate;
		}
		
		while ( true )
		{
			uchar_t packet[256];
			int bytes_read = connection.ReceivePacket( packet, sizeof(packet) );
			if ( bytes_read == 0 )
				break;
		}
		
		// show packets that were acked this frame
		
		#ifdef SHOW_ACKS
		uint32_t * acks = nullptr;
		uint32_t ack_count = connection.GetReliabilitySystem().GetAcks( &acks );
		if ( ack_count > 0 )
		{
			printf( "acks: %d", acks[0] );
			for ( uint32_t i = 1; i < ack_count; ++i )
				printf( ",%d", acks[i] );
			printf( "\n" );
		}
		#endif

		// update connection
		
		connection.Update( DeltaTime );

		// show connection stats
		
		statsAccumulator += DeltaTime;

		while ( statsAccumulator >= 0.25f && connection.IsConnected() )
		{
			float rtt = connection.GetReliabilitySystem().GetRoundTripTime();
			
			uint32_t sent_packets = connection.GetReliabilitySystem().GetSentPackets();
			uint32_t acked_packets = connection.GetReliabilitySystem().GetAckedPackets();
			uint32_t lost_packets = connection.GetReliabilitySystem().GetLostPackets();
			
			float sent_bandwidth = connection.GetReliabilitySystem().GetSentBandwidth();
			float acked_bandwidth = connection.GetReliabilitySystem().GetAckedBandwidth();
			
			printf( "rtt %.1fms, sent %d, acked %d, lost %d (%.1f%%), sent bandwidth = %.1fkbps, acked bandwidth = %.1fkbps\n", 
				rtt * 1000.0f, sent_packets, acked_packets, lost_packets, 
				sent_packets > 0.0f ? (float) lost_packets / (float) sent_packets * 100.0f : 0.0f, 
				sent_bandwidth, acked_bandwidth );
			
			statsAccumulator -= 0.25f;
		}
		// c++11 sleep 1 ms
		this_thread::sleep_for(DeltaTimeInMs);
		// net::wait( DeltaTime );
	}
	
	ShutdownSockets();

	return 0;
}
