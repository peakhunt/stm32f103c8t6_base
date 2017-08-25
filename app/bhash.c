//
// a simple generic bucket hash implementation
//
// all rights reserved, Intellian Technologies, 2012
//
// Revision History
// - Oct/22/2012 initial release by hkim
//
//
#include <stdlib.h>
#include <string.h>
#include "bhash.h"

static uint32_t
djb_hash(uint8_t *key, int32_t len )
{
   uint32_t   h = 0;
   int32_t    i;
   
   for( i = 0; i < len; i++ )
   {
      h = 33 * h ^ key[i];
   }
   return h;
}


#if 0
static uint32_t
calc_hash(uint8_t* key, int32_t key_size)
{
   uint8_t  *p = key;
   uint32_t h = 0, g;
   int32_t  i;
   
   for ( i = 0; i < key_size; i++ )
   {
      h = ( h << 4 ) + p[i];
      g = h & 0xf0000000L; 
      if ( g != 0 )
         h ^= g >> 24;
      h &= ~g;
   }
   return h;

#if 0
   unsigned int hash = 0;
   int i;

   for(i = 0; i < key_size; i++)
   {
      hash = key[i] + (hash << 6) + (hash << 16) - hash;
   }
   return hash;
#endif
}
#endif

/**
 * initialize a hash context
 *
 * @param hash hash context block
 * @param numBuckets number of buckets to use for this hash context
 * @param hash_offset offset of hash element
 * @param key_offset offset of hash key
 * @param key_size size of hash key to use
 * @param func hash function to use, use default if NULL
 */
void
bhash_init(BHashContext* hash, struct list_head* buckets, int32_t numBuckets, int32_t hash_offset,
      int32_t key_offset, int32_t key_size, hash_func func)
{
  int32_t i ;

  hash->numBuckets  = numBuckets;
  hash->offset      = hash_offset;
  hash->key_offset  = key_offset;
  hash->key_size    = key_size;
  hash->buckets     = buckets;

  if(func != NULL)
  {
    hash->calc_hash = func;
  }
  else
  {
    hash->calc_hash   = djb_hash;
  }

  for(i = 0; i < numBuckets; i++)
  {
    INIT_LIST_HEAD(&hash->buckets[i]);
  }
}

/**
 * deinitialize hash context block
 */
void
bhash_deinit(BHashContext* hash)
{
}

/**
 * lookup hash for a given key
 *
 * @param hash hash context block
 * @param key key to search with
 * @return NULL when key is not found, pointer to client structure when found
 */
void*
bhash_lookup(BHashContext* hash, void* key)
{
  uint32_t ndx;
  struct list_head* pos;

  ndx = hash->calc_hash(key, hash->key_size) % hash->numBuckets % hash->numBuckets;

  list_for_each(pos, &hash->buckets[ndx])
  {
    int8_t* key2;

    key2 = (int8_t*)pos - hash->offset + hash->key_offset;

    if(memcmp(key, key2, hash->key_size) == 0)
    {
      return (int8_t*)pos - hash->offset;
    }
  }
  return NULL;
}

/**
 * add a new client element to hash
 *
 * @param hash hash context block
 * @param element hash client element
 * @return 0 on failure, 1 on success
 */
int32_t
bhash_add(BHashContext* hash, void* element)
{
  uint32_t   ndx;
  int8_t*    key;
  struct list_head* lh;

  key = (int8_t*)element + hash->key_offset;
  lh = (struct list_head*)((int8_t*)element + hash->offset);

  if(bhash_lookup(hash, key) != NULL)
  {
    return 0;
  }

  ndx = hash->calc_hash((uint8_t*)key, hash->key_size) % hash->numBuckets;

  list_add_tail(lh, &hash->buckets[ndx]);
  return 1;
}

/**
 * delete a hash client element with given key from hash context
 *
 * @param hash hash context block
 * @param key hash key to search with
 * @return 1 on success, 0 on failure
 */
int32_t
bhash_del(BHashContext* hash, void* key)
{
  uint32_t ndx;
  struct list_head* pos;

  if(bhash_lookup(hash, key) == NULL)
  {
    return 0;
  }

  ndx = hash->calc_hash((uint8_t*)key, hash->key_size) % hash->numBuckets;

  list_for_each(pos, &hash->buckets[ndx])
  {
    int8_t* key2;

    key2 = (int8_t*)pos - hash->offset + hash->key_offset;

    if(memcmp(key, key2, hash->key_size) == 0)
    {
      list_del(pos);
      return 1;
    }
  }
  return 0;
}
