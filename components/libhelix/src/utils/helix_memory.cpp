#include "Allocator.h"
#include "helix_log.h"
#include "ConfigHelix.h"
#include <esp_heap_caps.h>

// MJB char log_buffer_helix[HELIX_LOG_SIZE];
// MJB ALLOCATOR alloc;

#ifdef __cplusplus
extern "C" {
#endif
/* MJB 
void* helix_malloc(int size) { return alloc.allocate(size); }

void helix_free(void* ptr) { alloc.free(ptr); }
*/

void* helix_malloc(int size) { 
	return heap_caps_malloc (size,MALLOC_CAP_SPIRAM);  
}

void helix_free(void* ptr) { free(ptr); }

#ifdef __cplusplus
}
#endif
