#include "mathid.h"
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <inttypes.h>

#define MAX_PAGE 256
#define PAGE_SIZE 1024

struct page {
	float v[PAGE_SIZE][4];
};

struct marked_count {
	uint8_t count[PAGE_SIZE];	
};

struct marked_freelist {
	struct marked_freelist *next;
	int page;
	int size;
};

struct math_ref {
	const float * ptr;
	int size;
	int type;
};

struct math_context {
	struct page * transient[MAX_PAGE];
	struct page * marked[MAX_PAGE];
	struct marked_count *count[MAX_PAGE];
	struct marked_freelist *freelist;
	int frame;
	int n;
	int marked_page;
};

static inline int
check_size(int size) {
	struct math_id s;
	s.size = ~0;
	return size > 0 && (size-1) <= s.size;
}

struct math_context *
math_new() {
	struct math_context * m = (struct math_context *)malloc(sizeof(*m));
	m->frame = 0;
	m->n = 0;
	m->marked_page = 0;
	m->freelist = NULL;
	m->marked[0] = NULL;
	m->count[0] = NULL;
	m->transient[0] = NULL;
	return m;
}

void
math_delete(struct math_context *M) {
	if (M == NULL)
		return;
	int i;
	for (i=0;i<MAX_PAGE;i++) {
		if (M->transient[i] == NULL) {
			break;
		}
		free(M->transient[i]);
	}
	for (i=0;i<MAX_PAGE;i++) {
		if (M->marked[i] == NULL) {
			break;
		}
		free(M->marked[i]);
	}
	for (i=0;i<MAX_PAGE;i++) {
		if (M->count[i] == NULL) {
			break;
		}
		free(M->count[i]);
	}
	free(M);
}

size_t
math_memsize(struct math_context *M) {
	size_t sz = sizeof(*M);
	int i;
	for (i=0;i<MAX_PAGE;i++) {
		if (M->transient[i] == NULL) {
			break;
		}
		sz += sizeof(struct page);
	}
	for (i=0;i<MAX_PAGE;i++) {
		if (M->marked[i] == NULL) {
			break;
		}
		sz += sizeof(struct page);
	}
	for (i=0;i<MAX_PAGE;i++) {
		if (M->count[i] == NULL) {
			break;
		}
		sz += sizeof(struct marked_count);
	}
	return sz;
}

static void *
allocvec(struct math_context *M, int size, int *index) {
	int page_id = M->n / PAGE_SIZE;
	int next_page_id = (M->n + size - 1) / PAGE_SIZE;
	if (next_page_id != page_id) {
		assert(next_page_id < MAX_PAGE);
		page_id = next_page_id;
		M->n = page_id * PAGE_SIZE;
	}
	if (M->transient[page_id] == NULL) {
		M->transient[page_id] = (struct page *)malloc(sizeof(struct page));
		if (page_id + 1 < MAX_PAGE) {
			M->transient[page_id+1] = NULL;
		}
	}
	*index = M->n;
	M->n += size;
	return M->transient[page_id]->v[*index % PAGE_SIZE];
}

static inline int
import(struct math_context *M, const float *v, int size) {
	int index;
	void * ptr = allocvec(M, size, &index);
	if (v) {
		memcpy(ptr, v, size * 4 * sizeof(float));
	}

	return index;
}

math_t
math_import(struct math_context *M, const float *v, int type, int size) {
	union {
		math_t id;
		struct math_id s;
	} u;
	assert(check_size(size));
	switch (type) {
	case MATH_TYPE_NULL:
		return MATH_NULL;
	case MATH_TYPE_MAT:
		u.s.index = import(M, v, 4 * size);
		break;
	case MATH_TYPE_VEC4:
	case MATH_TYPE_QUAT:
		u.s.index = import(M, v, 1 * size);
		break;
	default:
		assert(0);
	}
	u.s.frame = M->frame;
	u.s.type = type;
	u.s.size = size - 1;
	u.s.transient = 1;
	return u.id;
}

math_t
math_ref(struct math_context *M, const float *v, int type, int size) {
	union {
		math_t id;
		struct math_id s;
	} u;
	assert(check_size(size));
	int index;
	struct math_ref * r = (struct math_ref *)allocvec(M, 1, &index);
	r->ptr = v;
	r->size = size;
	assert(type == MATH_TYPE_MAT || type == MATH_TYPE_VEC4 || type == MATH_TYPE_QUAT);
	r->type = type;

	u.s.index = index;
	u.s.frame = M->frame;
	u.s.type = MATH_TYPE_REF;
	u.s.size = 0;
	u.s.transient = 1;
	return u.id;
}

float *
get_transient(struct math_context *M, int index) {
	assert(index < M->n);
	int page_id = index / PAGE_SIZE;
	return M->transient[page_id]->v[index % PAGE_SIZE];
}

float *
get_reference(struct math_context *M, int index, int offset) {
	struct math_ref * r = (struct math_ref *)get_transient(M, index);
	assert(offset >= 0 && offset < r->size);
	float * base = (float *)r->ptr;
	if (r->type == MATH_TYPE_MAT) {
		return base + 16 * offset;
	} else {
		return base + 4 * offset;
	}
}

int
math_ref_size_(struct math_context *M, struct math_id id) {
	struct math_ref * r = (struct math_ref *)get_transient(M, id.index);
	return r->size;
}

int
math_ref_type_(struct math_context *M, struct math_id id) {
	struct math_ref * r = (struct math_ref *)get_transient(M, id.index);
	return r->type;
}

static int
check_marked(struct math_context *M, int index, int size) {
	int page_id = index / PAGE_SIZE;
	if (page_id >= M->marked_page)
		return 0;
	index %= PAGE_SIZE;
	if (index + size > PAGE_SIZE)
		return 0;
	return M->count[page_id]->count[index] > 0;
}

int
math_valid(struct math_context *M, math_t id) {
	union {
		math_t id;
		struct math_id s;
	} u;
	u.id = id;
	if (u.s.transient) {
		return M->frame == u.s.frame;
	} else {
		if (u.s.frame == 0)
			return 1;
		if (u.s.frame != 1)
			return 0;
		// marked
		switch (u.s.type) {
		case MATH_TYPE_MAT:
			return check_marked(M, u.s.index, 4 * (u.s.size + 1));
		case MATH_TYPE_VEC4:
		case MATH_TYPE_QUAT:
			return check_marked(M, u.s.index, u.s.size + 1);
		default:
			return 0;
		}
	}
}

float *
get_marked(struct math_context *M, int index) {
	int page_id = index / PAGE_SIZE;
	index %= PAGE_SIZE;
	assert (page_id < M->marked_page);
	return M->marked[page_id]->v[index];
}

math_t
math_index(struct math_context *M, math_t id, int index) {
	union {
		math_t id;
		struct math_id s;
	} u;
	u.id = id;
	int size = math_size(M, id);
	assert(index < size);
	if (u.s.type == MATH_TYPE_REF) {
		u.s.size = index;
		return u.id;
	}
	u.s.size = 0;
	u.s.index += index;
	return u.id;
}

float *
math_value(struct math_context *M, math_t id) {
	union {
		math_t id;
		struct math_id s;
	} u;
	u.id = id;
	if (u.s.transient) {
		assert(u.s.frame == M->frame);
		if (u.s.type == MATH_TYPE_REF) {
			return get_reference(M, u.s.index, u.s.size);
		}
		return get_transient(M, u.s.index);
	} else {
		return get_marked(M, u.s.index);
	}
}

static struct marked_freelist *
new_marked_page(struct math_context *M) {
	assert (M->marked_page < MAX_PAGE);
	int page = M->marked_page++;
	if (M->marked_page < MAX_PAGE) {
		M->marked[M->marked_page] = NULL;
		M->count[M->marked_page] = NULL;
		++M->marked_page;
	}
	assert(M->marked[page] == NULL);
	M->marked[page] = (struct page *)malloc(sizeof(struct page));
	assert(M->count[page] == NULL);
	M->count[page] = (struct marked_count *)malloc(sizeof(struct marked_count));
	memset(M->count[page], 0, sizeof(struct marked_count));

	struct marked_freelist *node = (struct marked_freelist *)M->marked[page];
	node->next = M->freelist;
	node->page = page;
	node->size = PAGE_SIZE;
	M->freelist = node;
	return node;
}

static int
alloc_vecarray(struct math_context *M, int vecsize) {
	struct marked_freelist **prev = &M->freelist;
	float * mem = NULL;
	int page_id = 0;
	for (;;) {
		struct marked_freelist *node = *prev;
		if (node == NULL) {
			node = new_marked_page(M);
		}
		if (node->size < vecsize) {
			prev = &node->next;
			node = node->next;
		} else if (node->size == vecsize) {
			// use this node
			*prev = node->next;
			mem = (float *)node;
			page_id = node->page;
			break;
		} else {
			// split this node
			assert(sizeof(*node) <= sizeof(float) * 4);
			mem = (float *)node + (node->size - vecsize) * 4;
			node->size -= vecsize;
			page_id = node->page;
			break;
		}
	}
	struct page *p = M->marked[page_id];
	int index = (mem - &p->v[0][0]) / 4;
	return index + page_id * PAGE_SIZE;
}

static math_t
alloc_marked(struct math_context *M, const float *v, int type, int size) {
	union {
		math_t id;
		struct math_id s;
	} u;
	
	int vecsize = size;
	if (type == MATH_TYPE_MAT)
		vecsize *= 4;

	int index = alloc_vecarray(M, vecsize);

	u.s.index = index;
	u.s.size = size - 1;
	u.s.frame = 1;
	u.s.type = type;
	u.s.transient = 0;

	float * ptr = get_marked(M, index);
	memcpy(ptr, v, vecsize * 4 * sizeof(float));

	int page_id = index / PAGE_SIZE;
	index %= PAGE_SIZE;
	M->count[page_id]->count[index] = 1;
	int i;
	for (i=1;i<vecsize;i++) {
		M->count[page_id]->count[index+i] = 255;
	}

	return u.id;
}


static math_t
get_marked_id(struct math_context *M, math_t id) {
	union {
		math_t id;
		struct math_id s;
	} u;
	u.id = id;
	int index = u.s.index;
	int page_id = index / PAGE_SIZE;
	index %= PAGE_SIZE;
	assert (page_id < M->marked_page);
	int count = M->count[page_id]->count[index];
	if (count == 255) {
		float *v = math_value(M, id);
		int size = math_size(M, id);
		return alloc_marked(M, v, u.s.type, size);
	} else {
		// add reference count
		++M->count[page_id]->count[index];
		return id;
	}
}

math_t
math_mark(struct math_context *M, math_t id) {
	union {
		math_t id;
		struct math_id s;
	} u;
	u.id = id;
	if (u.s.transient) {
		float *v = math_value(M, id);
		int size = math_size(M, id);
		int type = math_type(M, id);
		return alloc_marked(M, v, type, size);
	}
	if (u.s.frame == 0) {
		// constant value
		return id;
	}
	return get_marked_id(M, id);
}

void
math_unmark(struct math_context *M, math_t id) {
	union {
		math_t id;
		struct math_id s;
	} u;
	u.id = id;
	assert(u.s.transient == 0);
	if (u.s.frame == 0)
		return;
	int index = u.s.index;
	int page_id = index / PAGE_SIZE;
	index %= PAGE_SIZE;
	assert(page_id < M->marked_page);
	int vecsize = u.s.size + 1;
	if (u.s.type == MATH_TYPE_MAT) {
		vecsize *= 4;
	}
	assert(vecsize + index <= PAGE_SIZE);
	uint8_t * count = &M->count[page_id]->count[index];
	int c = *count;
	assert(c > 0);
	if (c == 1) {
		// The last reference
		struct marked_freelist * node = (struct marked_freelist *)math_value(M, id);
		int i;
		for (i=1;i<vecsize;i++) {
			count[i] = 0;
		}
		node->next = M->freelist;
		node->size = vecsize;
		node->page = page_id;
		M->freelist = node;
	}
	*count = c - 1;
}

void
math_frame(struct math_context *M) {
	union {
		math_t id;
		struct math_id s;
	} u;
	memset(&u, 0xff, sizeof(u));
	++M->frame;
	if (M->frame > u.s.frame) {
		M->frame = 0;
	}
	int i;
	for (i=(M->n / PAGE_SIZE) + 1; i < MAX_PAGE; i ++) {
		if (M->transient[i] == NULL)
			break;
		free(M->transient[i]);
		M->transient[i] = NULL;
	}
	M->n = 0;
}

#include <stdio.h>

void
math_print(struct math_context *M, math_t id) {
	const float * v = math_value(M, id);
	int type = math_type(M, id);
	int size = math_size(M, id);
	int n = 0;
	union {
		math_t id;
		struct math_id s;
	} u;
	u.id = id;
	switch (type) {
	case MATH_TYPE_NULL:
		printf("[NULL]\n");
		return;
	case MATH_TYPE_MAT:
		printf("[MAT (%" PRIx64 , id.idx);
		n = 16;
		break;
	case MATH_TYPE_VEC4:
		printf("[VEC4 (%" PRIx64 , id.idx);
		n = 4;
		break;
	case MATH_TYPE_QUAT:
		printf("[QUAT (%" PRIx64 , id.idx);
		n = 4;
		break;
	default:
		printf("[INVALID (%" PRIx64 "]\n", id.idx);
		return;
	}
	if (u.s.transient) {
		printf(") :");
	} else {
		int index = u.s.index;
		int page = index / PAGE_SIZE;
		index %= PAGE_SIZE;
		int c = M->count[page]->count[index];
		printf("/%d) :", c);
	}
	if (size == 1 || u.s.type == MATH_TYPE_REF) {
		int i;
		if ( u.s.type == MATH_TYPE_REF ) {
			printf(" <%d/%d>", u.s.size, size);
		}
		for (i=0;i<n;i++) {
			printf(" %g", v[i]);
		}
	} else {
		int i,j;
		for (i=0;i<size;i++) {
			printf(" <%d/%d>", i, size);
			for (j=0;j<n;j++) {
				printf(" %g", *v);
				++v;
			}
		}
	}
	printf("]\n");
}

#ifdef TEST_MATHID

int
main() {
	struct math_context *M = math_new();
	float v[4] = { 1,2,3,4 };
	float array[3][4] = {
		{ 1, 0, 0, 0 },
		{ 2, 0, 0, 0 },
		{ 3, 0, 0, 0 },
	};
	float stack[2][16] = {
		{ 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16},
		{ 15,14,13,12,11,10,9,8,7,6,5,4,3,2,1},
	};
	math_t p[7];
	p[0] = math_vec4(M, v);
	p[1] = math_import(M, NULL, MATH_TYPE_QUAT, 3);
	float *buf = math_value(M, p[1]);
	memcpy(buf, &array[0][0], sizeof(array));
	p[2] = math_index(M, p[1], 2);
	p[3] = math_ref(M, &stack[0][0], MATH_TYPE_MAT, 2);
	p[4] = math_index(M, p[3], 1);
	p[5] = math_mark(M, p[0]);

	int i;
	for (i=0;i<6;i++) {
		printf("%d:", i);
		math_print(M, p[i]);
	}
	math_unmark(M, p[5]);
	p[6] = math_mark(M, p[3]);
	math_print(M, p[6]);

	for (i=0;i<7;i++) {
		printf("%d : %s\n", i, math_valid(M, p[i]) ? "valid" : "invalid");
	}

	math_frame(M);

	for (i=0;i<7;i++) {
		printf("%d : %s\n", i, math_valid(M, p[i]) ? "valid" : "invalid");
	}

	printf("mem : %d\n", (int)math_memsize(M));

	math_delete(M);
	return 0;
}

#endif
