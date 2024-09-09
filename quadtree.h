#ifndef QUADTREE_H
#define QUADTREE_H

#include "linear_algebra.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Below sets the max depth of quad tree.
#define QT_MAX_DEPTH 10

typedef struct {
  Vec2 pos;
  Vec2 half_dim;
} Boundary;

bool BoundaryContains(Boundary *b, Vec2 pos) {
  // look at x position, check that it falls within the boundary. then,
  // look at the y position and do the same.
  return (pos.x >= (b->pos.x - b->half_dim.x) &&
          pos.x <= (b->pos.x + b->half_dim.x)) &&
         (pos.y >= (b->pos.y - b->half_dim.y) &&
          pos.y <= (b->pos.y + b->half_dim.y));
}
bool BoundaryIntersects(Boundary *this, Boundary *that) {
  // We check to see if space exists between the x and y coordinates
  // Because of center x, y orientation we take the difference, but need
  // to compare it both widths or heights of this and that.
  if (fabs(this->pos.x - that->pos.x) > (this->half_dim.x + that->half_dim.x)) {
    return false;
  }
  if (fabs(this->pos.y - that->pos.y) > (this->half_dim.y + that->half_dim.y)) {
    return false;
  }
  // If both are false, we have an intersection.
  return true;
}
typedef struct {
  void *data;
  Vec2 pos;
} QT_Object;

typedef struct QuadTree {
  Boundary boundary;
  QT_Object *objects;
  int capacity;
  int obj_count;
  int depth;
  struct QuadTree *branches[4];
} QuadTree;

QuadTree *QT_New(Boundary b, int capacity) {
  QuadTree *qt = malloc(sizeof(QuadTree));
  if (qt == NULL) {
    fprintf(stderr, "Memory allocation for QuadTree failed\n");
    exit(EXIT_FAILURE);
  }
  qt->boundary = b;
  qt->objects = malloc(capacity * sizeof(QT_Object));
  if (qt->objects == NULL) {
    fprintf(stderr, "Memory allocation for QT_Object array failed\n");
    free(qt);
    exit(EXIT_FAILURE);
  }
  qt->capacity = capacity;
  qt->depth = 1;
  qt->obj_count = 0;
  for (int i = 0; i < 4; i++) {
    qt->branches[i] = NULL;
  }
  return qt;
}

bool QT_IsLeaf(QuadTree *qt) { return qt->branches[0] == NULL; }

// Forward declaration of QT_Insert. Defined after Subdivide, the two
// function have the ability to call each other.
bool QT_Insert(QuadTree *qt, QT_Object *obj);

void QT_Subdivide(QuadTree *qt) {
  float x = qt->boundary.pos.x;
  float y = qt->boundary.pos.y;
  float child_half_width = qt->boundary.half_dim.x / 2.0f;
  float child_half_height = qt->boundary.half_dim.y / 2.0f;
  Boundary b_northwest = {(Vec2){qt->boundary.pos.x - child_half_width,
                                 qt->boundary.pos.y - child_half_height},
                          (Vec2){child_half_width, child_half_height}};
  Boundary b_southwest = {(Vec2){qt->boundary.pos.x - child_half_width,
                                 qt->boundary.pos.y + child_half_height},
                          (Vec2){child_half_width, child_half_height}};
  Boundary b_northeast = {(Vec2){qt->boundary.pos.x + child_half_width,
                                 qt->boundary.pos.y - child_half_height},
                          (Vec2){child_half_width, child_half_height}};
  Boundary b_southeast = {(Vec2){qt->boundary.pos.x + child_half_width,
                                 qt->boundary.pos.y + child_half_height},
                          (Vec2){child_half_width, child_half_height}};
  qt->branches[0] = QT_New(b_northwest, qt->capacity);
  qt->branches[0]->depth = qt->depth + 1;
  qt->branches[1] = QT_New(b_southwest, qt->capacity);
  qt->branches[1]->depth = qt->depth + 1;
  qt->branches[2] = QT_New(b_northeast, qt->capacity);
  qt->branches[2]->depth = qt->depth + 1;
  qt->branches[3] = QT_New(b_southeast, qt->capacity);
  qt->branches[3]->depth = qt->depth + 1;
  // push down data.
  for (int objIdx = 0; objIdx < qt->obj_count; objIdx++) {
    for (int i = 0; i < 4; i++) {
      if (QT_Insert(qt->branches[i], &qt->objects[objIdx])) {
        // TO DO: Should I decrement qt variable obj_count here?
        break;
      }
    }
  }
  // clear parent node objects and set capacity to zero.
  qt->obj_count = 0;
  free(qt->objects);
  qt->objects = NULL;
}

bool QT_Insert(QuadTree *qt, QT_Object *obj) {
  if (!BoundaryContains(&qt->boundary, obj->pos)) {
    return false;
  }
  // NOTE: in the below we need the NULL check so that we are not inserting
  // in parent node after a subdivision.
  if (qt->obj_count < qt->capacity && qt->objects != NULL) {
    qt->objects[qt->obj_count++] = *obj;
    return true;
  }
  if (QT_IsLeaf(qt) && (qt->depth <= QT_MAX_DEPTH)) {
    QT_Subdivide(qt);
    for (int i = 0; i < 4; i++) {
      if (QT_Insert(qt->branches[i], obj)) {
        return true;
      }
    }
  }
  // If we reach the following, max depth is reached and we need to realloc the
  // objects. I am making decision to double the capacity to limit the amount of
  // malloc calls when building the quadtree.
  printf("Reached maximum depth, reallocating objects array\n");
  qt->capacity *= 2; // Double the capacity
  qt->objects = realloc(qt->objects, qt->capacity * sizeof(QT_Object));
  if (qt->objects == NULL) {
    fprintf(stderr, "Memory reallocation for QT_Object array failed\n");
    exit(EXIT_FAILURE);
  }
  qt->objects[qt->obj_count++] = *obj;
  return true;
}

bool QT_Query(QuadTree *qt, Boundary *query, int *resultCount,
              QT_Object *resultArray) {
  if (!BoundaryIntersects(&qt->boundary, query)) {
    return false;
  }
  for (int i = 0; i < qt->obj_count; i++) {
    QT_Object *obj = &qt->objects[i];
    if (BoundaryContains(query, obj->pos)) {
      memcpy(&resultArray[*resultCount], obj, sizeof(QT_Object));
      (*resultCount)++;
    }
  }
  // Recurse through the tree if QuadTree is not a leaf (endpoint of tree).
  if (!QT_IsLeaf(qt)) {
    for (int i = 0; i < 4; i++) {
      QT_Query(qt->branches[i], query, resultCount, resultArray);
    }
  }
  return true;
}

void QT_Destroy(QuadTree *root) {
  if (root == NULL)
    return;
  if (!QT_IsLeaf(root)) {
    for (int i = 0; i < 4; i++) {
      QT_Destroy(root->branches[i]);
    }
  }
  if (root->objects != NULL) {
    free(root->objects);
    root->objects = NULL;
  }
  free(root);
  root = NULL;
}

#endif // QUADTREE_H
