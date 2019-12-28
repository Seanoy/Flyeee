//#include "rtos_test.h"

////使用两个队列创建邮箱，一个用作寄信人，一个用作派送员
///* A mailbox can hold a fixed size data item. The size of the data item is set
//when the mailbox (queue) is created. In this example the mailbox is created to
//hold an Example_t structure. Example_t includes a time stamp to allow the data held
//in the mailbox to note the time at which the mailbox was last updated. The time
//stamp used in this example is for demonstration purposes only - a mailbox can hold
//any data the application writer wants, and the data does not need to include a time
//stamp. */

//typedef struct xExampleStructure
//{
//    TickType_t xTimeStamp;//时间戳
//    uint32_t ulValue;//数据
//} Example_t;//用于演示的结构体，实际上数据并不需要带时间戳
///* A mailbox is a queue, so its handle is stored in a variable of type
//QueueHandle_t. */
//Example_t example;
//QueueHandle_t xMailbox;
//void vAFunction( void )
//{
//    /* Create the queue that is going to be used as a mailbox. The queue has a
//    length of 1 to allow it to be used with the xQueueOverwrite() API function, which
//    is described below. */
//    xMailbox = xQueueCreate( 1, sizeof( Example_t ) );
//}

//void vUpdateMailbox( uint32_t ulNewValue )
//{
//    /* Example_t was defined in Listing 67. */
//    Example_t xData;
//    /* Write the new data into the Example_t structure.*/
//    xData.ulValue = ulNewValue;
//    /* Use the RTOS tick count as the time stamp stored in the Example_t structure. */
//    xData.xTimeStamp = xTaskGetTickCount();
//    /* Send the structure to the mailbox - overwriting any data that is already in the
//    mailbox. */
//    xQueueOverwrite( xMailbox, &xData );
//}

//BaseType_t vReadMailbox( Example_t *pxData )
//{
//    TickType_t xPreviousTimeStamp;
//    BaseType_t xDataUpdated;
//    /* This function updates an Example_t structure with the latest value received
//    from the mailbox. Record the time stamp already contained in *pxData before it
//    gets overwritten by the new data. */
//    xPreviousTimeStamp = pxData->xTimeStamp;
//    /* Update the Example_t structure pointed to by pxData with the data contained in
//    the mailbox. If xQueueReceive() was used here then the mailbox would be left
//    empty, and the data could not then be read by any other tasks. Using
//    xQueuePeek() instead of xQueueReceive() ensures the data remains in the mailbox.
//    A block time is specified, so the calling task will be placed in the Blocked
//    state to wait for the mailbox to contain data should the mailbox be empty. An
//    infinite block time is used, so it is not necessary to check the value returned
//    from xQueuePeek(), as xQueuePeek() will only return when data is available. */
//    xQueuePeek( xMailbox, pxData, portMAX_DELAY );
//    /* Return pdTRUE if the value read from the mailbox has been updated since this
//    function was last called. Otherwise return pdFALSE. */
//    if( pxData->xTimeStamp > xPreviousTimeStamp )
//    {
//        xDataUpdated = pdTRUE;
//    }
//    else
//    {
//        xDataUpdated = pdFALSE;
//    }
//    return xDataUpdated;
//}


//static void vSenderTask( void *pvParameters )
//{
//    int32_t lValueToSend;
//    /* Two instances of this task are created so the value that is sent to the
//    queue is passed in via the task parameter - this way each instance can use
//    a different value. The queue was created to hold values of type int32_t,
//    so cast the parameter to the required type. */
//    lValueToSend = ( int32_t ) pvParameters;//该参数在创建任务的时候已经传递了，在使用的时候可以根据需求赋值到此变量
//    /* As per most tasks, this task is implemented within an infinite loop. */
//    const TickType_t xTicksToWait = pdMS_TO_TICKS( 200 );

//    for( ;; )
//    {
//        /* Send the value to the queue.
//        The first parameter is the queue to which data is being sent. The
//        queue was created before the scheduler was started, so before this task
//        started to execute.
//        The second parameter is the address of the data to be sent, in this case
//        the address of lValueToSend.
//        The third parameter is the Block time – the time the task should be kept
//        in the Blocked state to wait for space to become available on the queue
//        should the queue already be full. In this case a block time is not
//        specified because the queue should never contain more than one item, and
//        therefore never be full. */
//        vUpdateMailbox(lValueToSend);
//        vTaskDelay(xTicksToWait);
//    }
//}

//static void vReaderTask( void *pvParameters )
//{

//    /* As per most tasks, this task is implemented within an infinite loop. */
//    const TickType_t xTicksToWait = pdMS_TO_TICKS( 200 );

//    for( ;; )
//    {
//        /* Send the value to the queue.
//        The first parameter is the queue to which data is being sent. The
//        queue was created before the scheduler was started, so before this task
//        started to execute.
//        The second parameter is the address of the data to be sent, in this case
//        the address of lValueToSend.
//        The third parameter is the Block time – the time the task should be kept
//        in the Blocked state to wait for space to become available on the queue
//        should the queue already be full. In this case a block time is not
//        specified because the queue should never contain more than one item, and
//        therefore never be full. */
//        vReadMailbox(&example);
//        vPrintStringAndNumber( "TimeStamp = %d Received = %d\r\n", example.xTimeStamp, example.ulValue );
//        vTaskDelay(xTicksToWait);
//    }
//}

//int main( void )
//{
//    BSP_Init();
//    /* Create the two queues, both of which send character pointers. The priority
//    of the receiving task is above the priority of the sending tasks, so the queues
//    will never have more than one item in them at any one time*/
//    vAFunction();
//    /* Create the queue set. Two queues will be added to the set, each of which can
//    contain 1 item, so the maximum number of queue handles the queue set will ever
//    have to hold at one time is 2 (2 queues multiplied by 1 item per queue). */

//    /* Create the tasks that send to the queues. */
//    xTaskCreate( vSenderTask, "Sender", 1000, ( void * ) 666, 1, NULL );
//    xTaskCreate( vReaderTask, "Reader", 1000, NULL, 2, NULL );

//    /* Start the scheduler so the created tasks start executing. */
//    vTaskStartScheduler();
//    /* As normal, vTaskStartScheduler() should not return, so the following lines
//    Will never execute. */
//    for( ;; );
//    return 0;
//}



///*
//void vFunction( void *pvParameters )
// {
// QueueHandle_t xQueue;
// uint32_t ulVarToSend, ulValReceived;

//	// Create a queue to hold one uint32_t value.  It is strongly
//	// recommended *not* to use xQueueOverwrite() on queues that can
//	// contain more than one value, and doing so will trigger an assertion
//	// if configASSERT() is defined.
//	xQueue = xQueueCreate( 1, sizeof( uint32_t ) );

//	// Write the value 10 to the queue using xQueueOverwrite().
//	ulVarToSend = 10;
//	xQueueOverwrite( xQueue, &ulVarToSend );

//	// Peeking the queue should now return 10, but leave the value 10 in
//	// the queue.  A block time of zero is used as it is known that the
//	// queue holds a value.
//	ulValReceived = 0;
//	xQueuePeek( xQueue, &ulValReceived, 0 );

//	if( ulValReceived != 10 )
//	{
//		// Error unless the item was removed by a different task.
//	}

//	// The queue is still full.  Use xQueueOverwrite() to overwrite the
//	// value held in the queue with 100.
//	ulVarToSend = 100;
//	xQueueOverwrite( xQueue, &ulVarToSend );

//	// This time read from the queue, leaving the queue empty once more.
//	// A block time of 0 is used again.
//	xQueueReceive( xQueue, &ulValReceived, 0 );

//	// The value read should be the last value written, even though the
//	// queue was already full when the value was written.
//	if( ulValReceived != 100 )
//	{
//		// Error!
//	}

//	// ...
//}*/

