#include <assert.h>
#include <stdlib.h>
#include <ucontext.h>
#include "thread.h"
#include "interrupt.h"

/* This is the wait queue structure */
struct wait_queue {
	/* ... Fill this in Lab 3 ... */
};

/* This is the thread control block */
enum{
	READY=0,
	RUNNING=1,
	EXIT=2,	
};

enum{
	ASSIGNED =1,
	AVAILABLE = 0,
};

struct thread {
	Tid threadID;
	int state;
	int yieldFlag;
	int exitFlag;
	ucontext_t context;
	int kernelThreadFlag;
	void *stackAddress;
};
typedef struct thread thread;

struct queueNode{
	struct thread *threadData;
	struct queueNode *next;
};
typedef struct queueNode queueNode;

struct threadQueue{
	queueNode *head;
};
typedef struct threadQueue threadQueue;

//Global Variables
int tidArray[THREAD_MAX_THREADS]; 
threadQueue *readyQueue;
threadQueue *exitQueue;
thread *runningThread;

//Helper Functions
//

void printQueue(int q){
	int queuePosition = 1;
	if(q == READY){
		queueNode *printNode = readyQueue->head;
		if(printNode == NULL){
			printf("Ready queue is empty.\n");
		}
		while(printNode != NULL){
			printf("Thread in position %d is %d.\n", queuePosition, printNode->threadData->threadID);
			printNode = printNode->next;
			queuePosition++;	
		}
	}
	else if(q == EXIT){
		queueNode *printNode = exitQueue->head;
		if(printNode == NULL){
			printf("Exit queue is empty.\n");
		}
		while(printNode != NULL){
			printf("Thread in position %d is %d.\n", queuePosition, printNode->threadData->threadID);
			printNode = printNode->next;
			queuePosition++;
		}
	}
}


//Find thread with Tid = TID in the given queue
int find(int q,Tid TID)
{
	//if q=READY,find in readyQueue
	//if q=EXIT, find in exitQueue
	
      	if(q == READY){
		queueNode *searchNode = readyQueue->head;
		//printf("Finding Tid: %d in readyQueue.\n", TID);
		if(searchNode == NULL){
			//printf("ReadyQueue is empty\n");
		}
		while(searchNode != NULL){
			//printf("Current checked node threadID: %d \n",searchNode->threadData->threadID);
			if(searchNode->threadData->threadID == TID){
				//printf("Found thread with tid %d in readyQueue.\n", TID);
				return 1;
			}
			searchNode = searchNode->next;
		}
		//Unsuccessful, thread with given Tid not found in queue
		return 0;
	}
	else if(q == EXIT){
		queueNode *searchNode = exitQueue->head;
		while(searchNode != NULL){
			if(searchNode->threadData->threadID == TID)
				return 1;
			searchNode = searchNode->next;	
		}
		//Unsuccessful, thread with given Tid not found in queue
		return 0;
	}
	else
		return 0;
}

//Find thread with Tid = TID and extract the data from the node
thread *extract(int q, Tid TID)
{
	thread *returnThread = NULL;
	if(q == READY){
		queueNode *extractNode = readyQueue->head;
		queueNode *prevNode = NULL;
		while(extractNode != NULL && extractNode->threadData->threadID != TID){
			prevNode = extractNode;
			extractNode = extractNode->next;
		}
		if(extractNode == NULL){
			returnThread = NULL;
		}
		else if(extractNode == readyQueue->head){
			returnThread = extractNode->threadData;
			readyQueue->head = extractNode->next;
			//printf("Found node to extract at head.\n");	
		}
		else if(extractNode->next == NULL){
			returnThread = extractNode->threadData;
			prevNode->next = NULL;
			//printf("Found node to extract at last position.\n");
		}
		else{
			returnThread = extractNode->threadData;
			prevNode->next = extractNode->next;
			//printf("Found node to extract in queue\n");
		}
		free(extractNode);
		//printQueue(READY);
	}
	else if(q == EXIT){
		queueNode *extractNode = exitQueue->head;
		queueNode *prevNode = NULL;
		while(extractNode != NULL && extractNode->threadData->threadID != TID){
			prevNode = extractNode;
			extractNode = extractNode->next;
		}
		if(extractNode == NULL){
			returnThread = NULL;
		}
		else if(extractNode == exitQueue->head){
			returnThread = extractNode->threadData;
			exitQueue->head = extractNode->next; 
		}
		else if(extractNode->next == NULL){
			returnThread = extractNode->threadData;
			prevNode->next = NULL;
		}
		else{
			returnThread = extractNode->threadData;
			prevNode->next = extractNode->next;
		}
		free(extractNode);
	}		
	return returnThread;
}

//Extracting the first node in the queue
thread *extractFirst(int q)
{
	thread *returnThread = NULL;
	if(q == READY && readyQueue->head != NULL){	
		queueNode *extractNode = readyQueue->head;
	       	returnThread = extractNode->threadData;
		readyQueue->head = extractNode->next;
		free(extractNode);
	}
	else if(q == EXIT && exitQueue->head != NULL){
		queueNode *extractNode = exitQueue->head;
		returnThread = extractNode->threadData;
		exitQueue->head = extractNode->next;
		free(extractNode);
	}	
	return returnThread;
}
queueNode*  queueNode_init(thread* threadData){
	//initialize new node;
	queueNode *newNode;
	newNode = (queueNode*)malloc(sizeof(queueNode));
	newNode->next=NULL;
	newNode->threadData = threadData;
	//printf("initialised newnode in appendqueue with data.\n");

	return newNode;
}

void appendQueue(int q, queueNode* appendNode)
{	
	//Add to end of queue
	if(q == READY){
		if(readyQueue->head ==NULL)
			readyQueue->head = appendNode;
		else{
			queueNode *current;
			current = readyQueue->head;
				while(current->next != NULL){
					current = current->next;
				}
				//printf("In appendQueue, reached end of queue.\n");
				current->next = appendNode;
		}
	}
	else if(q == EXIT){	
		if(exitQueue->head ==NULL)
			exitQueue->head = appendNode;
		else{
			queueNode *current;
			current = exitQueue->head;
			
			while(current->next != NULL){
				current = current->next;
			}
			//printf("In appendQueue, reached end of queue.\n");
			current->next = appendNode;
		}
	}
}

void clearQueue(int q){
	if(q==READY){
		while(readyQueue->head!=NULL){
			queueNode *destroyNode = readyQueue->head;
			readyQueue->head = destroyNode->next;
			tidArray[destroyNode->threadData->threadID]=AVAILABLE;
			//free(destroyNode->threadData);
			free(destroyNode);
		}
	}
	else if(q==EXIT){
		thread *destroyThread = extractFirst(EXIT);
		
		while(destroyThread != NULL){
			//printf("Destorying thread %d.\n", destroyThread->threadID);
			tidArray[destroyThread->threadID] = AVAILABLE;
			free(destroyThread->stackAddress);
			free(destroyThread);
			destroyThread = extractFirst(EXIT);
			}
	}
}

//Cooperative Threads API
void
thread_init(void)
{
	/* your optional code here */	
	readyQueue = (threadQueue *)malloc(sizeof(threadQueue));
	exitQueue = (threadQueue *)malloc(sizeof(threadQueue));
	readyQueue->head = NULL;
	exitQueue->head = NULL;
	//printf("Allocated lists for ready and exit queue");
	for(int i=0;i<THREAD_MAX_THREADS;i++)
		tidArray[i]=AVAILABLE;
	
	//set kernel thread
	thread *kernelThread = (thread*)malloc(sizeof(thread));
	kernelThread->threadID = 0;
	kernelThread->state = RUNNING;
	kernelThread->yieldFlag = 0;
	kernelThread->exitFlag = 0;
	kernelThread->kernelThreadFlag=1;
       	tidArray[0] = ASSIGNED;	
	//printf("Kernel thread set");
	
	//initialize running thread
	runningThread = kernelThread;	
}

Tid
thread_id()
{
	return runningThread->threadID;
}

void thread_stub(void (*thread_main)(void *), void *arg)
{
	Tid ret;
	thread_main(arg);
	ret = thread_exit();
	//we should only get here if we are the last thread
	assert(ret == THREAD_NONE);
	//all threads are done, so process should exit
	exit(0);
}

Tid
thread_create(void (*fn) (void *), void *parg)
{
	thread *newThread = (thread *)malloc(sizeof(thread));
	if(newThread == NULL){
		return THREAD_NOMEMORY;
	}

	//Have memory for thread, find available thread ID	
	int i;
	for(i=0;tidArray[i] == ASSIGNED;i++){
		if(i==THREAD_MAX_THREADS-1)
			return THREAD_NOMORE;
	}
	newThread->threadID = i;
	tidArray[i]=1;
	//printf("Creating new thread with ID: %d.\n", newThread->threadID);

	//Initialize temp->context, then update temp context
	getcontext(&newThread->context);
	//printf("Getting current context for new thread number: %d.\n", newThread->threadID);
	
	newThread->context.uc_mcontext.gregs[REG_RIP] = (long long int) &thread_stub;
	newThread->context.uc_mcontext.gregs[REG_RDI] = (long long int) fn;
	newThread->context.uc_mcontext.gregs[REG_RSI] = (long long int) parg;
	//printf("Updated context for new thread number: %d.\n", newThread->threadID);

	newThread->state = READY;
	newThread->yieldFlag = 0;
	newThread->exitFlag = 0;
	newThread->kernelThreadFlag=0;
	
	void *stackPointer;
	stackPointer = malloc(THREAD_MIN_STACK);
	if(stackPointer == NULL){
		free(newThread);
		return THREAD_NOMEMORY;
	}
	//printf("Allocated memory for stack pointer, threadID: %d.\n", newThread->threadID);

	newThread->context.uc_stack.ss_sp = stackPointer;
	newThread->context.uc_mcontext.gregs[REG_RSP] = (long long int)stackPointer + THREAD_MIN_STACK - 8;
	newThread->context.uc_stack.ss_size = THREAD_MIN_STACK - 8;
	newThread->stackAddress = stackPointer;
	//printf("Updated more context for thread number: %d.\n", newThread->threadID);
	
	queueNode* newNode = queueNode_init(newThread);
	appendQueue(READY, newNode);

	//printf("Added thread %d to ready queue.\n", newThread->threadID);

	return newThread->threadID;
}

Tid
thread_yield(Tid want_tid)
{	
	//Checking if want_tid refers to a valid thread
	if(want_tid < -2 || want_tid > THREAD_MAX_THREADS)
		return THREAD_INVALID;
	else{
		if(want_tid == THREAD_ANY && readyQueue->head == NULL){
			clearQueue(EXIT);
			return THREAD_NONE;
		}
		if(want_tid >= 0 && tidArray[want_tid] == AVAILABLE)
			return THREAD_INVALID;
	}	
	//printf("In thread_yield, Tid checks passed.\n");

	//Yield to self
	if(want_tid == THREAD_SELF || (want_tid>=0 && want_tid == runningThread->threadID)){
		//printf("thread_yield yielding to self, tid: %d: \n", runningThread->threadID);
		clearQueue(EXIT);
		return runningThread->threadID;
	}
	
	//Yield to another thread
	Tid ret;
	if(runningThread->exitFlag==0){
		queueNode* appendNode = queueNode_init(runningThread);
		appendQueue(READY, appendNode);
		//printf("In thread_yield, current running thread(TID %d) stopped and added to readyQueue.\n", runningThread->threadID);
		getcontext(&appendNode->threadData->context);
		//printf("Current context stored\n");
	}
	else if(runningThread->exitFlag == 1){
	}

	else
		return THREAD_INVALID;

	if(runningThread->yieldFlag ==0){
		//First set yieldFlag
		runningThread->yieldFlag = 1;

		if(want_tid == THREAD_ANY){	
			//If TID value is THREAD_ANY(-1), yield to first thread in readyQueue
			runningThread = extractFirst(READY);
			//printf("In thread_yield, yielding to any thread. New running thread: %d\n",runningThread->threadID);
		}
		else if(want_tid >= 0 && want_tid != runningThread->threadID){
			//Find wanted_tid in readyQueue 
			//printf("thread_yield yielding to thread: %d, current running tid: %d: \n", want_tid, runningThread->threadID);
			if(find(READY,want_tid) == 1){
				//printf("In thread_yield, tid %d found, getting current context to store in runningThread.\n", want_tid);	
				runningThread = extract(READY, want_tid);
				//printf("In thread_yield, replacementThread(TID %d) extracted from readyQueue.\n", runningThread->threadID);
			}	
			else{
				//printf("Can't find tid %d in readyQueue.\n", want_tid);
				return THREAD_INVALID;
			}
		}
		//If wanted_tid not in readyQueue
		else
			return THREAD_INVALID;
		//printf("In thread_yield, finished setting context of thread %d.\n", runningThread->threadID);
		
		ret = runningThread->threadID;
		setcontext(&runningThread->context);
	}
	
	//reset yield flag to zero
	runningThread->yieldFlag = 0;
	
	//clear Exit queue to kill exit threads
	
	clearQueue(EXIT);
	return ret;
}

Tid
thread_exit()
{
	//Check if last thread
	if (readyQueue->head == NULL)
		return THREAD_NONE;
	else{
		//move current thread to exitQueue
		queueNode* appendNode = queueNode_init(runningThread);
		appendQueue(EXIT, appendNode);
		//Flag to exit at yield below
		runningThread->exitFlag = 1;
		
		thread *destroyThread = extractFirst(EXIT);
	
		while(destroyThread != NULL){
			//printf("Destorying thread %d.\n", destroyThread->threadID);
			tidArray[destroyThread->threadID] = AVAILABLE;
			free(destroyThread->stackAddress);
			free(destroyThread);
			destroyThread = extractFirst(EXIT);
		}
		//printf("Exiting thread: %d.\n", runningThread->threadID);
		//yield to any thread
		thread_yield(THREAD_ANY);
	}
	return THREAD_FAILED;
}

Tid
thread_kill(Tid tid)
{
	//Checking if want_tid refers to a valid thread
	if(tid < -2 || tid > THREAD_MAX_THREADS)
		return THREAD_INVALID;
	else{
		if(tid > 0 && tidArray[tid] == AVAILABLE)
			return THREAD_INVALID;
	}
	
	//Check passed
	//printQueue(EXIT);
	//checking if killing yourself
	
	if(tid == runningThread->threadID){
		return THREAD_INVALID;
	}
	else if(find(READY,tid) == 1){
		thread *killThread; //might need to be pointer
		killThread = extract(READY, tid);
		queueNode* appendNode = queueNode_init(killThread);
		appendQueue(EXIT, appendNode);
		return killThread->threadID;
	}	
	return THREAD_FAILED;
}

/*******************************************************************
 * Important: The rest of the code should be implemented in Lab 3. *
 *******************************************************************/

/* make sure to fill the wait_queue structure defined above */
struct wait_queue *
wait_queue_create()
{
	struct wait_queue *wq;

	wq = malloc(sizeof(struct wait_queue));
	assert(wq);

	TBD();

	return wq;
}

void
wait_queue_destroy(struct wait_queue *wq)
{
	TBD();
	free(wq);
}

Tid
thread_sleep(struct wait_queue *queue)
{
	TBD();
	return THREAD_FAILED;
}

/* when the 'all' parameter is 1, wakeup all threads waiting in the queue.
 * returns whether a thread was woken up on not. */
int
thread_wakeup(struct wait_queue *queue, int all)
{
	TBD();
	return 0;
}

/* suspend current thread until Thread tid exits */
Tid
thread_wait(Tid tid)
{
	TBD();
	return 0;
}

struct lock {
	/* ... Fill this in ... */
};

struct lock *
lock_create()
{
	struct lock *lock;

	lock = malloc(sizeof(struct lock));
	assert(lock);

	TBD();

	return lock;
}

void
lock_destroy(struct lock *lock)
{
	assert(lock != NULL);

	TBD();

	free(lock);
}

void
lock_acquire(struct lock *lock)
{
	assert(lock != NULL);

	TBD();
}

void
lock_release(struct lock *lock)
{
	assert(lock != NULL);

	TBD();
}

struct cv {
	/* ... Fill this in ... */
};

struct cv *
cv_create()
{
	struct cv *cv;

	cv = malloc(sizeof(struct cv));
	assert(cv);

	TBD();

	return cv;
}

void
cv_destroy(struct cv *cv)
{
	assert(cv != NULL);

	TBD();

	free(cv);
}

void
cv_wait(struct cv *cv, struct lock *lock)
{
	assert(cv != NULL);
	assert(lock != NULL);

	TBD();
}

void
cv_signal(struct cv *cv, struct lock *lock)
{
	assert(cv != NULL);
	assert(lock != NULL);

	TBD();
}

void
cv_broadcast(struct cv *cv, struct lock *lock)
{
	assert(cv != NULL);
	assert(lock != NULL);

	TBD();
}
