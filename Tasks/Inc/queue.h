#ifndef __QUEUE_H
#define __QUEUE_H

#include<stdio.h>
#include<stdlib.h>
#include<includes.h>

#define SIZE 1000

typedef struct {
	int arr[SIZE];
	int head; //记录最前面数字所在的下标
	int tail; //记录最后一个有效数字的下一个坐标
			  //如果队列里一个数都没有的话head=tail
}Queue;

void queue_init(Queue *);
void queue_deinit(Queue *);
int queue_size(const Queue *);
int queue_empty(const Queue *);
int queue_full(const Queue *);
int queue_push(Queue *, int);
int queue_pop(Queue *, int *);
int queue_front(const Queue *, int *);

extern Queue AutoGet_Queue;
#endif /*__QUEUE_H*/
