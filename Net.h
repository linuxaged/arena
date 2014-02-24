#ifndef NET_H
#define NET_H

// platform detection

#define PLATFORM_WINDOWS  1
#define PLATFORM_MAC      2
#define PLATFORM_UNIX     3

#if defined(_WIN32)
#define PLATFORM PLATFORM_WINDOWS
#elif defined(__APPLE__)
#define PLATFORM PLATFORM_MAC
#else
#define PLATFORM PLATFORM_UNIX
#endif

#if PLATFORM == PLATFORM_WINDOWS

#include <winsock2.h>
#pragma comment( lib, "wsock32.lib" )

#elif PLATFORM == PLATFORM_MAC || PLATFORM == PLATFORM_UNIX

#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>

#else

#error unknown platform!

#endif
// 统一类型
// uchar_t
// int8_t
// int16_t
// int32_t
// int64_t
// uint8_t
// uint16_t
// uint32_t
// uint64_t
using uchar_t = unsigned char;
using SIZE_T = uint32_t;
static_assert(sizeof(float) == 4, "sizeof(float) == 4");
#include <cstdint>

#include <assert.h>
#include <vector>
#include <map>
#include <stack>
#include <list>
#include <algorithm> // std::copy
#include <functional>
#include <cstring>
namespace net
{
#include <unistd.h>
#include <netinet/in.h> // htonl

// internet address
class Address
{
public:

    Address()
    {
        _address = 0;
        _port = 0;
    }

    Address( uchar_t a, uchar_t b, uchar_t c, uchar_t d, unsigned short port )
    {
        this->_address = static_cast<uint32_t>(a<<24);
        this->_address |= static_cast<uint32_t>(b<<16);
        this->_address |= static_cast<uint32_t>(c<<8);
        this->_address |= static_cast<uint32_t>(d);
        printf("%x\n", this->_address);
        this->_port = port;
    }

    Address( uint32_t address, unsigned short port )
    {
        this->_address = address;
        this->_port = port;
    }

    uint32_t GetAddress() const
    {
        return _address;
    }

    uchar_t GetA() const
    {
        return static_cast<uchar_t>( _address >> 24 );
    }

    uchar_t GetB() const
    {
        return static_cast<uchar_t>( _address >> 16 );
    }

    uchar_t GetC() const
    {
        return static_cast<uchar_t>( _address >> 8 );
    }

    uchar_t GetD() const
    {
        return static_cast<uchar_t>( _address );
    }

    unsigned short GetPort() const
    {
        return _port;
    }

    bool operator == ( const Address &other ) const
    {
        return _address == other._address && _port == other._port;
    }

    bool operator != ( const Address &other ) const
    {
        return ! ( *this == other );
    }

    bool operator < ( const Address &other ) const
    {
        // note: this is so we can use address as a key in std::map
        if ( _address < other._address )
            return true;
        if ( _address > other._address )
            return false;
        else
            return _port < other._port;
    }

private:

    uint32_t _address;
    unsigned short _port;
};

// sockets

inline bool InitializeSockets()
{
#if PLATFORM == PLATFORM_WINDOWS
    WSADATA WsaData;
    return WSAStartup( MAKEWORD(2, 2), &WsaData ) != NO_ERROR;
#else
    return true;
#endif
}

inline void ShutdownSockets()
{
#if PLATFORM == PLATFORM_WINDOWS
    WSACleanup();
#endif
}

class Socket
{
public:

    Socket()
    {
        socket = 0;
    }

    ~Socket()
    {
        Close();
    }

    bool Open( unsigned short port )
    {
        assert( !IsOpen() );

        // create socket

        socket = ::socket( AF_INET, SOCK_DGRAM, IPPROTO_UDP );

        if ( socket <= 0 )
        {
            printf( "failed to create socket\n" );
            socket = 0;
            return false;
        }

        // bind to port

        sockaddr_in address;
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_ANY);
        address.sin_port = htons(static_cast<uint16_t>(port));

        if ( bind( socket, (sockaddr *)(&address), sizeof(sockaddr_in) ) < 0 )
        {
            printf( "failed to bind socket\n" );
            Close();
            return false;
        }

        // set non-blocking io

#if PLATFORM == PLATFORM_MAC || PLATFORM == PLATFORM_UNIX

        int nonBlocking = 1;
        if ( fcntl( socket, F_SETFL, O_NONBLOCK, nonBlocking ) == -1 )
        {
            printf( "failed to set non-blocking socket\n" );
            Close();
            return false;
        }

#elif PLATFORM == PLATFORM_WINDOWS

        DWORD nonBlocking = 1;
        if ( ioctlsocket( socket, FIONBIO, &nonBlocking ) != 0 )
        {
            printf( "failed to set non-blocking socket\n" );
            Close();
            return false;
        }

#endif

        return true;
    }

    void Close()
    {
        if ( socket != 0 )
        {
#if PLATFORM == PLATFORM_MAC || PLATFORM == PLATFORM_UNIX
            close( socket );
#elif PLATFORM == PLATFORM_WINDOWS
            closesocket( socket );
#endif
            socket = 0;
        }
    }

    bool IsOpen() const
    {
        return socket != 0;
    }

    bool Send( const Address &destination, const void *data, SIZE_T size )
    {
        assert( data );
        assert( size > 0 );

        if ( socket == 0 )
            return false;

        assert( destination.GetAddress() != 0 );
        assert( destination.GetPort() != 0 );

        sockaddr_in address;
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl( destination.GetAddress() );
        address.sin_port = htons( static_cast<uint16_t>(destination.GetPort()) );

        int sent_bytes = sendto( socket, data, size, 0, (sockaddr*)(&address), sizeof(sockaddr_in) );

        return sent_bytes == size;
    }

    int Receive( Address &sender, void *data, SIZE_T size )
    {
        assert( data );
        assert( size > 0 );

        if ( socket == 0 )
            return false;

#if PLATFORM == PLATFORM_WINDOWS
        typedef int socklen_t;
#endif

        sockaddr_in from;
        socklen_t fromLength = sizeof( from );

        int received_bytes = recvfrom( socket, data, size, 0, (sockaddr *)&from, &fromLength );

        if ( received_bytes <= 0 )
            return 0;

        uint32_t address = ntohl( from.sin_addr.s_addr );
        unsigned short port = ntohs( from.sin_port );

        sender = Address( address, port );

        return received_bytes;
    }

private:

    int socket;
};

// connection

class Connection
{
public:

    enum Mode
    {
        None,
        Client,
        Server
    };

    Connection( uint32_t pID, float TO )
    {
        this->protocolId = protocolId;
        this->timeout = TO;
        mode = None;
        running = false;
        ClearData();
    }

    virtual ~Connection()
    {
        if ( IsRunning() )
            Stop();
    }

    bool Start( unsigned short port )
    {
        assert( !running );
        printf( "start connection on port %d\n", port );
        if ( !socket.Open( port ) )
            return false;
        running = true;
        OnStart();
        return true;
    }

    void Stop()
    {
        assert( running );
        printf( "stop connection\n" );
        bool connected = IsConnected();
        ClearData();
        socket.Close();
        running = false;
        if ( connected )
            OnDisconnect();
        OnStop();
    }

    bool IsRunning() const
    {
        return running;
    }

    void Listen()
    {
        printf( "server listening for connection\n" );
        bool connected = IsConnected();
        ClearData();
        if ( connected )
            OnDisconnect();
        mode = Server;
        state = Listening;
    }

    void Connect( const Address &addr )
    {
        printf( "client connecting to %hhu.%hhu.%hhu.%hhu:%d\n",
                address.GetA(), address.GetB(), address.GetC(), address.GetD(), address.GetPort() );
        bool connected = IsConnected();
        ClearData();
        if ( connected )
            OnDisconnect();
        mode = Client;
        state = Connecting;
        this->address = addr;
    }

    bool IsConnecting() const
    {
        return state == Connecting;
    }

    bool ConnectFailed() const
    {
        return state == ConnectFail;
    }

    bool IsConnected() const
    {
        return state == Connected;
    }

    bool IsListening() const
    {
        return state == Listening;
    }

    Mode GetMode() const
    {
        return mode;
    }

    virtual void Update( float deltaTime )
    {
        assert( running );
        timeoutAccumulator += deltaTime;
        if ( timeoutAccumulator > timeout )
        {
            if ( state == Connecting )
            {
                printf( "connect timed out\n" );
                ClearData();
                state = ConnectFail;
                OnDisconnect();
            }
            else if ( state == Connected )
            {
                printf( "connection timed out\n" );
                ClearData();
                if ( state == Connecting )
                    state = ConnectFail;
                OnDisconnect();
            }
        }
    }
    // 添加4字节的 协议ID 后发送
    virtual bool SendPacket( const uchar_t data[], SIZE_T size )
    {
        assert( running );
        if ( address.GetAddress() == 0 )
            return false;
        uchar_t packet[size + 4];
        packet[0] = static_cast<uchar_t>( protocolId >> 24 );
        packet[1] = static_cast<uchar_t>( ( protocolId >> 16 ) & 0xFF );
        packet[2] = static_cast<uchar_t>( ( protocolId >> 8 ) & 0xFF );
        packet[3] = static_cast<uchar_t>( ( protocolId ) & 0xFF );
        #ifdef MEMCPY
        memcpy( &packet[4], data, size );
        #else
        std::copy( data, data + size, &packet[4] );
        #endif
        return socket.Send( address, packet, size + 4 );
    }

    virtual int ReceivePacket( uchar_t data[], SIZE_T size )
    {
        assert( running );
        uchar_t packet[size + 4];
        Address sender;
        int bytes_read = socket.Receive( sender, packet, size + 4 );
        if ( bytes_read == 0 )
            return 0;
        if ( bytes_read <= 4 )
            return 0;
        if ( packet[0] != static_cast<uchar_t>( protocolId >> 24 ) ||
                packet[1] != static_cast<uchar_t>( ( protocolId >> 16 ) & 0xFF ) ||
                packet[2] != static_cast<uchar_t>( ( protocolId >> 8 ) & 0xFF ) ||
                packet[3] != static_cast<uchar_t>( protocolId & 0xFF ) )
            return 0;
        if ( mode == Server && !IsConnected() )
        {
            printf( "server accepts connection from client %d.%d.%d.%d:%d\n",
                    sender.GetA(), sender.GetB(), sender.GetC(), sender.GetD(), sender.GetPort() );
            state = Connected;
            address = sender;
            OnConnect();
        }
        if ( sender == address )
        {
            if ( mode == Client && state == Connecting )
            {
                printf( "client completes connection with server\n" );
                state = Connected;
                OnConnect();
            }
            timeoutAccumulator = 0.0f;
            #ifdef MEMCPY
            memcpy( data, &packet[4], bytes_read - 4 );
            #else
            std::copy( &packet[4], &packet[4] + bytes_read - 4, data);
            #endif
            return bytes_read - 4;
        }
        return 0;
    }

    int GetHeaderSize() const
    {
        return 4;
    }

protected:

    virtual void OnStart()      {}
    virtual void OnStop()       {}
    virtual void OnConnect()    {}
    virtual void OnDisconnect() {}

private:

    void ClearData()
    {
        state = Disconnected;
        timeoutAccumulator = 0.0f;
        address = Address();
    }

    enum State
    {
        Disconnected,
        Listening,
        Connecting,
        ConnectFail,
        Connected
    };

    uint32_t protocolId;
    float timeout;

    bool running;
    Mode mode;
    State state;
    Socket socket;
    float timeoutAccumulator;
    Address address;
};

// packet queue to store information about sent and received packets sorted in sequence order
//  + we define ordering using the "sequence_more_recent" function, this works provided there is a large gap when sequence wrap occurs

struct PacketData
{
    uint32_t sequence;          // packet sequence number
    float time;                     // time offset since packet was sent or received (depending on context)
    SIZE_T size;                       // packet size in bytes
};

inline bool sequence_more_recent( uint32_t s1, uint32_t s2, uint32_t max_sequence )
{
    return (( s1 > s2 ) && ( s1 - s2 <= max_sequence / 2 )) || (( s2 > s1 ) && ( s2 - s1 > max_sequence / 2 ));
}

class PacketQueue : public std::list<PacketData>
{
public:

    bool exists( uint32_t sequence )
    {
        for ( iterator itor = begin(); itor != end(); ++itor )
            if ( itor->sequence == sequence )
                return true;
        return false;
    }

    void insert_sorted( const PacketData &p, uint32_t max_sequence )
    {
        if ( empty() )
        {
            push_back( p );
        }
        else
        {
            if ( !sequence_more_recent( p.sequence, front().sequence, max_sequence ) )
            {
                push_front( p );
            }
            else if ( sequence_more_recent( p.sequence, back().sequence, max_sequence ) )
            {
                push_back( p );
            }
            else
            {
                for ( PacketQueue::iterator itor = begin(); itor != end(); itor++ )
                {
                    assert( itor->sequence != p.sequence );
                    if ( sequence_more_recent( itor->sequence, p.sequence, max_sequence ) )
                    {
                        insert( itor, p );
                        break;
                    }
                }
            }
        }
    }

    void verify_sorted( uint32_t max_sequence )
    {
        PacketQueue::iterator prev = end();
        for ( PacketQueue::iterator itor = begin(); itor != end(); itor++ )
        {
            assert( itor->sequence <= max_sequence );
            if ( prev != end() )
            {
                assert( sequence_more_recent( itor->sequence, prev->sequence, max_sequence ) );
                prev = itor;
            }
        }
    }
};

// reliability system to support reliable connection
//  + manages sent, received, pending ack and acked packet queues
//  + separated out from reliable connection because it is quite complex and i want to unit test it!

class ReliabilitySystem
{
public:

    ReliabilitySystem( uint32_t max_sequence = 0xFFFFFFFF )
    {
        this->max_sequence = max_sequence;
        Reset();
    }

    void Reset()
    {
        local_sequence = 0;
        remote_sequence = 0;
        sentQueue.clear();
        receivedQueue.clear();
        pendingAckQueue.clear();
        ackedQueue.clear();
        sent_packets = 0;
        recv_packets = 0;
        lost_packets = 0;
        acked_packets = 0;
        sent_bandwidth = 0.0f;
        acked_bandwidth = 0.0f;
        rtt = 0.0f;
        rtt_maximum = 1.0f;
    }

    void PacketSent( SIZE_T size )
    {
        if ( sentQueue.exists( local_sequence ) )
        {
            printf( "local sequence %d exists\n", local_sequence );
            for ( PacketQueue::iterator itor = sentQueue.begin(); itor != sentQueue.end(); ++itor )
                printf( " + %d\n", itor->sequence );
        }
        assert( !sentQueue.exists( local_sequence ) );
        assert( !pendingAckQueue.exists( local_sequence ) );
        PacketData data;
        data.sequence = local_sequence;
        data.time = 0.0f;
        data.size = size;
        sentQueue.push_back( data );
        pendingAckQueue.push_back( data );
        sent_packets++;
        local_sequence++;
        if ( local_sequence > max_sequence )
            local_sequence = 0;
    }

    void PacketReceived( uint32_t sequence, SIZE_T size )
    {
        recv_packets++;
        if ( receivedQueue.exists( sequence ) )
            return;
        PacketData data;
        data.sequence = sequence;
        data.time = 0.0f;
        data.size = size;
        receivedQueue.push_back( data );
        if ( sequence_more_recent( sequence, remote_sequence, max_sequence ) )
            remote_sequence = sequence;
    }

    uint32_t GenerateAckBits()
    {
        return generate_ack_bits( GetRemoteSequence(), receivedQueue, max_sequence );
    }

    void ProcessAck( uint32_t ack, uint32_t ack_bits )
    {
        process_ack( ack, ack_bits, pendingAckQueue, ackedQueue, acks, acked_packets, rtt, max_sequence );
    }

    void Update( float deltaTime )
    {
        acks.clear();
        AdvanceQueueTime( deltaTime );
        UpdateQueues();
        UpdateStats();
#ifdef NET_UNIT_TEST
        Validate();
#endif
    }

    void Validate()
    {
        sentQueue.verify_sorted( max_sequence );
        receivedQueue.verify_sorted( max_sequence );
        pendingAckQueue.verify_sorted( max_sequence );
        ackedQueue.verify_sorted( max_sequence );
    }

    // utility functions

    static bool sequence_more_recent( uint32_t s1, uint32_t s2, uint32_t max_sequence )
    {
        return (( s1 > s2 ) && ( s1 - s2 <= max_sequence / 2 )) || (( s2 > s1 ) && ( s2 - s1 > max_sequence / 2 ));
    }

    static int bit_index_for_sequence( uint32_t sequence, uint32_t ack, uint32_t max_sequence )
    {
        assert( sequence != ack );
        assert( !sequence_more_recent( sequence, ack, max_sequence ) );
        if ( sequence > ack )
        {
            assert( ack < 33 );
            assert( max_sequence >= sequence );
            return ack + ( max_sequence - sequence );
        }
        else
        {
            assert( ack >= 1 );
            assert( sequence <= ack - 1 );
            return ack - 1 - sequence;
        }
    }

    static uint32_t generate_ack_bits( uint32_t ack, const PacketQueue &received_queue, uint32_t max_sequence )
    {
        uint32_t ack_bits = 0;
        for ( PacketQueue::const_iterator itor = received_queue.begin(); itor != received_queue.end(); itor++ )
        {
            if ( itor->sequence == ack || sequence_more_recent( itor->sequence, ack, max_sequence ) )
                break;
            int bit_index = bit_index_for_sequence( itor->sequence, ack, max_sequence );
            if ( bit_index <= 31 )
                ack_bits |= 1 << bit_index;
        }
        return ack_bits;
    }

    static void process_ack( uint32_t ack, uint32_t ack_bits,
                             PacketQueue &pending_ack_queue, PacketQueue &acked_queue,
                             std::vector<uint32_t> &acks, uint32_t &acked_packets,
                             float &rtt, uint32_t max_sequence )
    {
        if ( pending_ack_queue.empty() )
            return;

        PacketQueue::iterator itor = pending_ack_queue.begin();
        while ( itor != pending_ack_queue.end() )
        {
            bool acked = false;

            if ( itor->sequence == ack )
            {
                acked = true;
            }
            else if ( !sequence_more_recent( itor->sequence, ack, max_sequence ) )
            {
                int bit_index = bit_index_for_sequence( itor->sequence, ack, max_sequence );
                if ( bit_index <= 31 )
                    acked = ( ack_bits >> bit_index ) & 1;
            }

            if ( acked )
            {
                rtt += ( itor->time - rtt ) * 0.1f;

                acked_queue.insert_sorted( *itor, max_sequence );
                acks.push_back( itor->sequence );
                acked_packets++;
                itor = pending_ack_queue.erase( itor );
            }
            else
                ++itor;
        }
    }

    // data accessors

    uint32_t GetLocalSequence() const
    {
        return local_sequence;
    }

    uint32_t GetRemoteSequence() const
    {
        return remote_sequence;
    }

    uint32_t GetMaxSequence() const
    {
        return max_sequence;
    }

    void GetAcks( uint32_t **ACKs, uint32_t& ack_count )
    {
        *ACKs = &this->acks[0];
        ack_count = static_cast<uint32_t>(this->acks.size());
    }

    uint32_t GetSentPackets() const
    {
        return sent_packets;
    }

    uint32_t GetReceivedPackets() const
    {
        return recv_packets;
    }

    uint32_t GetLostPackets() const
    {
        return lost_packets;
    }

    uint32_t GetAckedPackets() const
    {
        return acked_packets;
    }

    float GetSentBandwidth() const
    {
        return sent_bandwidth;
    }

    float GetAckedBandwidth() const
    {
        return acked_bandwidth;
    }

    float GetRoundTripTime() const
    {
        return rtt;
    }

    int GetHeaderSize() const
    {
        return 12;
    }

protected:

    void AdvanceQueueTime( float deltaTime )
    {
        for ( PacketQueue::iterator itor = sentQueue.begin(); itor != sentQueue.end(); itor++ )
            itor->time += deltaTime;

        for ( PacketQueue::iterator itor = receivedQueue.begin(); itor != receivedQueue.end(); itor++ )
            itor->time += deltaTime;

        for ( PacketQueue::iterator itor = pendingAckQueue.begin(); itor != pendingAckQueue.end(); itor++ )
            itor->time += deltaTime;

        for ( PacketQueue::iterator itor = ackedQueue.begin(); itor != ackedQueue.end(); itor++ )
            itor->time += deltaTime;
    }

    void UpdateQueues()
    {
        const float epsilon = 0.001f;

        while ( sentQueue.size() && sentQueue.front().time > rtt_maximum + epsilon )
            sentQueue.pop_front();

        if ( receivedQueue.size() )
        {
            const uint32_t latest_sequence = receivedQueue.back().sequence;
            const uint32_t minimum_sequence = latest_sequence >= 34 ? ( latest_sequence - 34 ) : max_sequence - ( 34 - latest_sequence );
            while ( receivedQueue.size() && !sequence_more_recent( receivedQueue.front().sequence, minimum_sequence, max_sequence ) )
                receivedQueue.pop_front();
        }

        while ( ackedQueue.size() && ackedQueue.front().time > rtt_maximum * 2 - epsilon )
            ackedQueue.pop_front();

        while ( pendingAckQueue.size() && pendingAckQueue.front().time > rtt_maximum + epsilon )
        {
            pendingAckQueue.pop_front();
            lost_packets++;
        }
    }

    void UpdateStats()
    {
        int sent_bytes_per_second = 0;
        for ( PacketQueue::iterator itor = sentQueue.begin(); itor != sentQueue.end(); ++itor )
            sent_bytes_per_second += itor->size;
        int acked_packets_per_second = 0;
        int acked_bytes_per_second = 0;
        for ( PacketQueue::iterator itor = ackedQueue.begin(); itor != ackedQueue.end(); ++itor )
        {
            if ( itor->time >= rtt_maximum )
            {
                acked_packets_per_second++;
                acked_bytes_per_second += itor->size;
            }
        }
        sent_bytes_per_second /= rtt_maximum;
        acked_bytes_per_second /= rtt_maximum;
        sent_bandwidth = sent_bytes_per_second * ( 8 / 1000.0f );
        acked_bandwidth = acked_bytes_per_second * ( 8 / 1000.0f );
    }

private:

    uint32_t max_sequence;          // maximum sequence value before wrap around (used to test sequence wrap at low # values)
    uint32_t local_sequence;        // local sequence number for most recently sent packet
    uint32_t remote_sequence;       // remote sequence number for most recently received packet

    uint32_t sent_packets;          // total number of packets sent
    uint32_t recv_packets;          // total number of packets received
    uint32_t lost_packets;          // total number of packets lost
    uint32_t acked_packets;         // total number of packets acked

    float sent_bandwidth;               // approximate sent bandwidth over the last second
    float acked_bandwidth;              // approximate acked bandwidth over the last second
    float rtt;                          // estimated round trip time
    float rtt_maximum;                  // maximum expected round trip time (hard coded to one second for the moment)

    std::vector<uint32_t> acks;     // acked packets from last set of packet receives. cleared each update!

    PacketQueue sentQueue;              // sent packets used to calculate sent bandwidth (kept until rtt_maximum)
    PacketQueue pendingAckQueue;        // sent packets which have not been acked yet (kept until rtt_maximum * 2 )
    PacketQueue receivedQueue;          // received packets for determining acks to send (kept up to most recent recv sequence - 32)
    PacketQueue ackedQueue;             // acked packets (kept until rtt_maximum * 2)
};

// connection with reliability (seq/ack)

class ReliableConnection : public Connection
{
public:

    ReliableConnection( uint32_t protocolId, float timeout, uint32_t max_sequence = 0xFFFFFFFF )
        : Connection( protocolId, timeout ), reliabilitySystem( max_sequence )
    {
        ClearData();
#ifdef NET_UNIT_TEST
        packet_loss_mask = 0;
#endif
    }

    ~ReliableConnection()
    {
        if ( IsRunning() )
            Stop();
    }

    // overriden functions from "Connection"
    bool SendPacket( const uchar_t data[], SIZE_T size )
    {
#ifdef NET_UNIT_TEST
        if ( reliabilitySystem.GetLocalSequence() & packet_loss_mask )
        {
            reliabilitySystem.PacketSent( size );
            return true;
        }
#endif
        const int header = 12;
        uchar_t packet[header + size];
        uint32_t seq = reliabilitySystem.GetLocalSequence();
        uint32_t ack = reliabilitySystem.GetRemoteSequence();
        uint32_t ack_bits = reliabilitySystem.GenerateAckBits();
        WriteHeader( packet, seq, ack, ack_bits );
        #ifdef MEMCPY
        memcpy( packet + header, data, size );
        #else
        std::copy( data, data + size, &packet[header] );
        #endif
        if ( !Connection::SendPacket( packet, size + header ) )
            return false;
        reliabilitySystem.PacketSent( size );
        return true;
    }

    int ReceivePacket( uchar_t data[], SIZE_T size )
    {
        const int header = 12;
        if ( size <= header )
            return false;
        uchar_t packet[header + size];
        int received_bytes = Connection::ReceivePacket( packet, size + header );
        if ( received_bytes == 0 )
            return false;
        if ( received_bytes <= header )
            return false;
        uint32_t packet_sequence = 0;
        uint32_t packet_ack = 0;
        uint32_t packet_ack_bits = 0;
        ReadHeader( packet, packet_sequence, packet_ack, packet_ack_bits );
        reliabilitySystem.PacketReceived( packet_sequence, received_bytes - header );
        reliabilitySystem.ProcessAck( packet_ack, packet_ack_bits );
        #ifdef MEMCPY
        memcpy( data, packet + header, received_bytes - header );
        #else
        std::copy( packet + header, packet + received_bytes, data);
        #endif
        return received_bytes - header;
    }

    void Update( float deltaTime )
    {
        Connection::Update( deltaTime );
        reliabilitySystem.Update( deltaTime );
    }

    int GetHeaderSize() const
    {
        return Connection::GetHeaderSize() + reliabilitySystem.GetHeaderSize();
    }

    ReliabilitySystem &GetReliabilitySystem()
    {
        return reliabilitySystem;
    }

    // unit test controls

#ifdef NET_UNIT_TEST
    void SetPacketLossMask( uint32_t mask )
    {
        packet_loss_mask = mask;
    }
#endif

protected:

    void WriteInteger( uchar_t *data, uint32_t value )
    {
        data[0] = static_cast<uchar_t>( value >> 24 );
        data[1] = static_cast<uchar_t>( ( value >> 16 ) & 0xFF );
        data[2] = static_cast<uchar_t>( ( value >> 8 ) & 0xFF );
        data[3] = static_cast<uchar_t>( value & 0xFF );
    }

    void WriteHeader( uchar_t *header, uint32_t sequence, uint32_t ack, uint32_t ack_bits )
    {
        WriteInteger( header, sequence );
        WriteInteger( header + 4, ack );
        WriteInteger( header + 8, ack_bits );
    }

    void ReadInteger( const uchar_t *data, uint32_t &value )
    {
        value = ( (static_cast<uint32_t>(data[0] << 24)) |
                  (static_cast<uint32_t>(data[1] << 16)) |
                  (static_cast<uint32_t>(data[2] << 8 )) |
                  (static_cast<uint32_t>(data[3]      ))
                );
    }

    void ReadHeader( const uchar_t *header, uint32_t &sequence, uint32_t &ack, uint32_t &ack_bits )
    {
        ReadInteger( header, sequence );
        ReadInteger( header + 4, ack );
        ReadInteger( header + 8, ack_bits );
    }

    virtual void OnStop()
    {
        ClearData();
    }

    virtual void OnDisconnect()
    {
        ClearData();
    }

private:

    void ClearData()
    {
        reliabilitySystem.Reset();
    }

#ifdef NET_UNIT_TEST
    uint32_t packet_loss_mask;          // mask sequence number, if non-zero, drop packet - for unit test only
#endif

    ReliabilitySystem reliabilitySystem;    // reliability system: manages sequence numbers and acks, tracks network stats etc.
};
}

#endif
