
#include "includes.h"

Queue AutoGet_Queue;
int Queue_Buffer;
//���еĳ�ʼ������
void queue_init(Queue *p_queue) {
	p_queue->head = 0;
	p_queue->tail = 0;
}



//����������
void queue_deinit(Queue *p_queue) {
	for(int i=0;i<100;i++)
	{
		p_queue->arr[i]=0;
	}
	p_queue->head = 0;
	p_queue->tail = 0;
}

//�������ָ���
int queue_size(const Queue *p_queue) {
	return (p_queue->tail - p_queue->head);
}

//�ж϶����Ƿ�Ϊ��
int queue_empty(const Queue *p_queue) {
	return !(p_queue->tail - p_queue->head);
}

//�ж϶����Ƿ�����
int queue_full(const Queue *p_queue) {
	return p_queue->tail >= SIZE;//tail�������һ��SIZE-1ʹ�ú��ΪSIZE��Ϊ����Ҫ����
}

//��������������
int queue_push(Queue *p_queue, int val) {
	if (queue_full(p_queue)) {
		return 0;
	}
	else {
		p_queue->arr[p_queue->tail] = val;
		p_queue->tail++;
		return 1;//��ʾ�����ּӽ�ȥ��
	}
}

//�Ӷ����������֣�������ִӶ�����ɾ����
int queue_pop(Queue *p_queue, int *p_num) {
	if (queue_empty(p_queue)) {
		return 0;
	}
	else {
		*p_num = p_queue->arr[p_queue->head];//��ΪҪɾ���������ȸ�
		p_queue->head++;//��ȡ����������ȥ
		return 1;
	}
}

//�Ӷ����������֣���������ִӶ���ɾ����
int queue_front(const Queue *p_queue, int *p_num) {
	if (queue_empty(p_queue)) {
		return 0;
	}
	else {
		*p_num = p_queue->arr[p_queue->head];//��ε�����ͬһ����
		return 1;
	}
}
