

/// Default config

#define USE_CHECKSUM_RADIO 1

//#define INSE_CSMA_MODE                 0
//#define INSE_BCAST_INTERVAL          10000 * WISELIB_TIME_FACTOR

#if defined(SHAWN)
	#define INSE_ROOT_NODE_ID              0 // lowest ID in 1A+
#else
	#define INSE_ROOT_NODE_ID              47430 // lowest ID in 1A+
#endif

#define INSE_PERIOD                          20000 * WISELIB_TIME_FACTOR
#define INSE_MIN_TRANSFER_INTERVAL_LENGTH    3000 * WISELIB_TIME_FACTOR

//#define INSE_PERIOD                          5000 * WISELIB_TIME_FACTOR
//#define INSE_MIN_TRANSFER_INTERVAL_LENGTH    1000 * WISELIB_TIME_FACTOR

#define INSE_LINK_METRIC_ALPHA         (4.0 / 8.0)

#define INSE_START_WAIT                (5 * 60)
#define INSE_ESTIMATE_RTT              1
#define INSE_ESTIMATE_RTT_ALPHA        (1.0/8.0)

#define INSE_MAX_NEIGHBORS             30
#define INSE_MAX_SEMANTIC_ENTITIES     4
#define INSE_MAX_QUERIES               4
//#define INSE_BLOOM_FILTER_BITS        64


#define INSE_MESSAGE_TYPE_BEACON       0x40
#define INSE_MESSAGE_TYPE_BEACON_ACK   0x41

//#define INSE_MESSAGE_TYPE_STRING_INQUIRY            0x40
//#define INSE_MESSAGE_TYPE_STRING_INQUIRY_ANSWER     0x41
//#define INSE_MESSAGE_TYPE_STRING_ANYCAST            0x42
//#define INSE_MESSAGE_TYPE_ROW_ANYCAST               0x43
//#define INSE_MESSAGE_TYPE_INTERMEDIATE_RESULT       0x44

//#define INSE_MESSAGE_TYPE_OPPORTUNISTIC_RELIABLE    0x47
//#define INSE_MESSAGE_TYPE_TREE_STATE                0x48

// TupleStore config

#define USE_LIST_CONTAINER             0
#define USE_VECTOR_CONTAINER           1
#define USE_BLOCK_CONTAINER            0
#define TUPLE_CONTAINER_SIZE           30

#define USE_PRESCILLA_DICTIONARY       0
#define USE_TREE_DICTIONARY            1
#define USE_BLOCK_DICTIONARY           0
#define USE_NULL_DICTIONARY            0

// Restrictions

// Memory sizes, word sizes, tec..    

#define BITMAP_ALLOCATOR_RAM_SIZE      2048
#define BLOCK_CACHE_SIZE               2
#define BLOCK_CACHE_SPECIAL            1
#define BLOCK_CACHE_WRITE_THROUGH      1
#define BLOCK_CHUNK_SIZE               8
#define BLOCK_CHUNK_ADDRESS_TYPE       ::uint32_t

// Timing

#define WISELIB_TIME_FACTOR            1
//#define INSE_FORWARDING_MAP_BITS       2048
//#define INSE_FORWARDING_SLOT_LENGTH    (500 * 100)

// Message types



//#define INSE_MESSAGE_TYPE_TOKEN_RELIABLE            0x46

#if defined(CONTIKI_TARGET_MICAZ)
	#define CHECK_INVARIANTS           0
	#define WISELIB_DISABLE_DEBUG      1
	#define WISELIB_DISABLE_DEBUG_MESSAGES 1
	#define USE_VECTOR_CONTAINER       0
	#define USE_LIST_CONTAINER       0
	#define USE_TREE_DICTIONARY        1
	#define USE_INQP                   0
	#define INSE_USE_AGGREGATOR        0
	#define BITMAP_ALLOCATOR_RAM_SIZE  1024

#elif defined(ISENSE)
	#define INSE_USE_LINK_METRIC           0

	#define INSE_BLOOM_FILTER_BITS         8
	#define INSE_BCAST_INTERVAL            10000
	#define INQP_AGGREGATE_CHECK_INTERVAL  1000
	#define DISTRIBUTOR_DEBUG_STATE        1
	#define INSE_DEBUG_STATE               0
	#define INSE_DEBUG_TOKEN               1
	#define INSE_DEBUG_TOPOLOGY            0
	#define INSE_DEBUG_TREE                0
	#define INSE_ANYCAST_DEBUG_STATE       0
	#define INSE_ROW_COLLECTOR_DEBUG_STATE 0
	#define NAP_CONTROL_DEBUG_STATE        0
	#define NAP_CONTROL_DEBUG_ONOFF        0
	#define NAP_CONTROL_ALWAYS_ON          1
	#define RELIABLE_TRANSPORT_DEBUG_STATE 0
	#define WISELIB_DISABLE_DEBUG          1
	#define WISELIB_DISABLE_DEBUG_MESSAGES 1
	#define INSE_DEBUG_WARNING             0
	
	#define WISELIB_TIME_FACTOR            1
	#define INSE_MAX_NEIGHBORS             2
	#define WISELIB_MAX_NEIGHBORS          (INSE_MAX_NEIGHBORS)
	#define INSE_MAX_SEMANTIC_ENTITIES     2
	#define INSE_MAX_QUERIES               3

	#define INSE_CSMA_MODE                 1
	
	
#elif defined(CONTIKI_TARGET_sky)
	#define BITMAP_ALLOCATOR_RAM_SIZE      1024
	#define INSE_USE_AGGREGATOR            0
	#define USE_INQP                       0
	
	#define INSE_USE_LINK_METRIC           1

	#define CHECK_INVARIANTS               0
	#define DISTRIBUTOR_DEBUG_STATE        0
	#define INSE_DEBUG_STATE               0
	#define INSE_DEBUG_TOKEN               0
	#define INSE_DEBUG_TOPOLOGY            0
	#define INSE_DEBUG_FORWARDING          0
	#define INSE_DEBUG_TREE                0
	#define INSE_ANYCAST_DEBUG_STATE       0
	#define INSE_ROW_COLLECTOR_DEBUG_STATE 0
	#define NAP_CONTROL_DEBUG_STATE        1
	#define NAP_CONTROL_DEBUG_ONOFF        1
	#define RELIABLE_TRANSPORT_DEBUG_STATE 0
	#define WISELIB_DISABLE_DEBUG          1
	#define WISELIB_DISABLE_DEBUG_MESSAGES 1


	#define WISELIB_TIME_FACTOR            1
	//#define INSE_FORWARDING_MAP_BITS       512
	//#define INSE_FORWARDING_SLOT_LENGTH    2000 * WISELIB_TIME_FACTOR
	
	#define INSE_MAX_NEIGHBORS             32
	#define INSE_MAX_SEMANTIC_ENTITIES     2
	#define INSE_MAX_QUERIES               0

	#define CONTIKI_MAX_TIMERS             40

	#if APP_BLINK
		#define INSE_DEBUG_TOKEN               1
		#define INSE_DEBUG_TOPOLOGY            1
	#define INSE_DEBUG_STATE               0
	#define INSE_DEBUG_WARNING             1
	#define RELIABLE_TRANSPORT_DEBUG_STATE 1
	#endif


	
#elif defined(SHAWN)
	//#define INSE_USE_AGGREGATOR            0
	//#define USE_INQP                       0

	//#define CHECK_INVARIANTS               1
	//#define DISTRIBUTOR_DEBUG_STATE        1
	//#define INSE_DEBUG_STATE               1
	//#define INSE_DEBUG_TOKEN               1
	//#define INSE_DEBUG_TOPOLOGY            1
	//#define INSE_DEBUG_TREE                1
	//#define INSE_ANYCAST_DEBUG_STATE       1
	//#define INSE_ROW_COLLECTOR_DEBUG_STATE 1
	//#define NAP_CONTROL_DEBUG_STATE        1
	//#define NAP_CONTROL_DEBUG_ONOFF        1
	//#define RELIABLE_TRANSPORT_DEBUG_STATE 1
	//#define WISELIB_DISABLE_DEBUG          0
	//#define WISELIB_DISABLE_DEBUG_MESSAGES 0
	
	#undef WISELIB_TIME_FACTOR
	#define WISELIB_TIME_FACTOR            100

	//#define INSE_FORWARDING_MAP_BITS       512
	//#define INSE_FORWARDING_SLOT_LENGTH    1000 * WISELIB_TIME_FACTOR
	
	#undef INSE_MAX_NEIGHBORS
	#define INSE_MAX_NEIGHBORS             32

	#undef INSE_MAX_SEMANTIC_ENTITIES
	#define INSE_MAX_SEMANTIC_ENTITIES     10

	#undef INSE_MAX_QUERIES
	#define INSE_MAX_QUERIES               0
	
#endif
	
//#endif


// Allocator

#define NEED_ALLOCATOR 1
#define NEED_OWN_ALLOCATOR (defined(TINYOS) || defined(CONTIKI) || defined(CONTIKI_TARGET_MICAZ))

#include <external_interface/external_interface.h>
#include <external_interface/external_interface_testing.h>
typedef wiselib::OSMODEL Os;

#if NEED_ALLOCATOR
	#if NEED_OWN_ALLOCATOR
		#warning "Using BITMAP allocator"

		#include <util/allocators/bitmap_allocator.h>
		typedef wiselib::BitmapAllocator<Os, BITMAP_ALLOCATOR_RAM_SIZE> Allocator;
		Allocator allocator_;
		Allocator& get_allocator() { return allocator_; }
	#else
		#include <util/allocators/malloc_free_allocator.h>
		typedef wiselib::MallocFreeAllocator<Os> Allocator;
		Allocator allocator_;
		Allocator& get_allocator() { return allocator_; }
	#endif
#endif

// OS quirks

#if defined(TINYOS) || defined(CONTIKI)
	int strcmp(const char* a, const char* b) {
		for( ; *a && *b; a++, b++) {
			if(*a != *b) { return *b - *a; }
		}
		return *b - *a;
	}
#endif

#if (defined(TINYOS) || defined(CONTIKI)) && !defined(memset)
	#include <string.h>

	void* memset(void* p, int v, unsigned int n) {
		char *p2 = (char*)p;
		for(unsigned int i = 0; i < n; i++) { *p2++ = (char)v; }
		return p;
	}

	int memcmp(const void* s1, const void* s2, size_t n) {
		const char *p1 = (const char*)s1;
		const char *p2 = (const char*)s2;
		for(unsigned int i = 0; i < n; i++) {
			if(p1[i] != p2[i]) { return p1[i] - p2[i]; }
		}
		return 0;
	}

/*
	char *strncpy(char *dest, const char *src, size_t n) {
		const char *end = src + n;
		for( ; src < end && *src; src++, dest++) {
			*dest = *src;
		}
		if(src < end) { *dest = '\0'; }
		return dest;
	}
	*/
#endif

#if defined(ISENSE)
	void* malloc(size_t n) { return isense::malloc(n); }
	void free(void* p) { isense::free(p); }
#endif
	
#if defined(ISENSE) || defined(TINYOS) || defined(CONTIKI) || defined(CONTIKI_TARGET_MICAZ)
	#warning "assertions messages not implemented for this platform, disabling"
	//extern "C" void assert(int) { }
	#define assert(X)
#endif
	
#ifndef DBG
	#warning "debug messages not implemented for this platform, disabling"
	#define DBG(...)
#endif
	
#if defined(ISENSE)
	#include <external_interface/isense_monitor.h>
#endif
	
template<typename OsModel_P>
class NullMonitor {
	public:
		void init(typename OsModel_P::Debug* d) { debug_ = d; }
		
		void report() { }
		void report(const char *remark) { debug_->debug(remark); }
		int free() { return 666; }
		
		typename OsModel_P::Debug* debug_;
};
	
/* vim: set ts=4 sw=4 tw=78 noexpandtab :*/
