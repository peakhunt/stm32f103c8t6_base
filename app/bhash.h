//
// a simple generic bucket hash implementation
//
#ifndef __BHASH_DEF_H__
#define __BHASH_DEF_H__

#include <stdio.h>
#include <stdint.h>
#include "list.h"

/**
 * a structure for hash element
 */
typedef struct bhash_element
{
   struct list_head  lh;         /** a list for buck hash list management */
} BHashElement;

/**
 * common hash prototype 
 */
typedef uint32_t (*hash_func)(uint8_t* key, int32_t key_size);

/**
 * a hash context
 */
typedef struct hash_context
{
   int32_t            numBuckets;       /** number of buckets                  */
   int32_t            offset;           /** offset of hash element             */
   int32_t            key_offset;       /** hash key offset                    */
   int32_t            key_size;         /** hash key size                      */
   hash_func          calc_hash;        /** hash function to use               */
   struct list_head* buckets;           /** bucket list                        */
} BHashContext;

extern void bhash_init(BHashContext* hash, struct list_head* buckets, int32_t numBuckets,
    int32_t hash_offset, int32_t key_offset, int32_t key_size, hash_func func);
extern void bhash_deinit(BHashContext* hash);
extern int32_t bhash_add(BHashContext* hash, void* element);
extern void* bhash_lookup(BHashContext* hash, void* key);
extern int32_t bhash_del(BHashContext* hash, void* key);

#endif //!__BHASH_DEF_H__
