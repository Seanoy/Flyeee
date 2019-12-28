//#include "rtos_test.h"

///* The handle of the queue from which character pointers are received. */
//QueueHandle_t xCharPointerQueue;//传输指向字符型数据的指针的队列
///* The handle of the queue from which uint32_t values are received. */
//QueueHandle_t xUint32tQueue;//传输指向uint32_t类型数据的指针的队列
///* The handle of the binary semaphore. */
//SemaphoreHandle_t xBinarySemaphore;//传输指向二进制信号量的指针的队列
///* The queue set to which the two queues and the binary semaphore belong. */
//QueueSetHandle_t xQueueSet;

//void vSenderTaskChar( void *pvParameters )
//{
//    const TickType_t xBlockTime = pdMS_TO_TICKS( 300 );
//    const char * const pcMessage = "Message from vSenderTaskChar\r\n";
//    /* As per most tasks, this task is implemented within an infinite loop. */
//    for( ;; )
//    {
//        /* Block for 100ms. */
//        vTaskDelay( xBlockTime );
//        /* Send this task's string to xQueue1. It is not necessary to use a block
//        time, even though the queue can only hold one item. This is because the
//        priority of the task that reads from the queue is higher than the priority of
//        this task; as soon as this task writes to the queue it will be pre-empted by
//        the task that reads from the queue, so the queue will already be empty again
//        by the time the call to xQueueSend() returns. The block time is set to 0. */
//        xQueueSend( xCharPointerQueue, &pcMessage, 0 );
//    }
//}
///*-----------------------------------------------------------*/
//void vSenderTaskUInt32( void *pvParameters )
//{
//    const TickType_t xBlockTime = pdMS_TO_TICKS( 300 );
//    const char * const pcMessage = "Message from vSenderTaskUInit32\r\n";
//    /* As per most tasks, this task is implemented within an infinite loop. */
//    for( ;; )
//    {
//        /* Block for 200ms. */
//        vTaskDelay( xBlockTime );
//        /* Send this task's string to xQueue2. It is not necessary to use a block
//        time, even though the queue can only hold one item. This is because the
//        priority of the task that reads from the queue is higher than the priority of
//        this task; as soon as this task writes to the queue it will be pre-empted by
//        the task that reads from the queue, so the queue will already be empty again
//        by the time the call to xQueueSend() returns. The block time is set to 0. */
//        xQueueSend( xUint32tQueue, &pcMessage, 0 );
//    }
//}

//void vSenderTaskBinary( void *pvParameters )
//{
//    const TickType_t xBlockTime = pdMS_TO_TICKS( 300 );
//    const char * const pcMessage = "Message from vSenderTaskBinary\r\n";
//    /* As per most tasks, this task is implemented within an infinite loop. */
//    for( ;; )
//    {
//        /* Block for 200ms. */
//        vTaskDelay( xBlockTime );
//        /* Send this task's string to xQueue2. It is not necessary to use a block
//        time, even though the queue can only hold one item. This is because the
//        priority of the task that reads from the queue is higher than the priority of
//        this task; as soon as this task writes to the queue it will be pre-empted by
//        the task that reads from the queue, so the queue will already be empty again
//        by the time the call to xQueueSend() returns. The block time is set to 0. */
//        xQueueSend( xBinarySemaphore, &pcMessage, 0 );
//    }
//}

//void vAMoreRealisticReceiverTask( void *pvParameters )
//{
//    QueueSetMemberHandle_t xHandle;
//    char *pcReceivedString;
//    uint32_t ulRecievedValue;
//    const TickType_t xDelay100ms = pdMS_TO_TICKS( 100 );
//    for( ;; )
//    {
//        /* Block on the queue set for a maximum of 100ms to wait for one of the members of
//        the set to contain data. */
//        xHandle = xQueueSelectFromSet( xQueueSet, xDelay100ms );
//        /* Test the value returned from xQueueSelectFromSet(). If the returned value is
//        NULL then the call to xQueueSelectFromSet() timed out. If the returned value is not
//        NULL then the returned value will be the handle of one of the set’s members. The
//        QueueSetMemberHandle_t value can be cast to either a QueueHandle_t or a
//        SemaphoreHandle_t. Whether an explicit cast is required depends on the compiler. */
//        if( xHandle == NULL )
//        {
//            /* The call to xQueueSelectFromSet() timed out. */
//        }
//        else if( xHandle == ( QueueSetMemberHandle_t ) xCharPointerQueue )
//        {
//            /* The call to xQueueSelectFromSet() returned the handle of the queue that
//            receives character pointers. Read from the queue. The queue is known to contain
//            data, so a block time of 0 is used. */
//            xQueueReceive( xCharPointerQueue, &pcReceivedString, 0 );
//            /* The received character pointer can be processed here... */
//        }
//        else if( xHandle == ( QueueSetMemberHandle_t ) xUint32tQueue )
//        {
//            /* The call to xQueueSelectFromSet() returned the handle of the queue that
//            receives uint32_t types. Read from the queue. The queue is known to contain
//            data, so a block time of 0 is used. */
//            xQueueReceive(xUint32tQueue, &ulRecievedValue, 0 );
//            /* The received value can be processed here... */
//        }
//        else if( xHandle == ( QueueSetMemberHandle_t ) xBinarySemaphore )
//        {
//            /* The call to xQueueSelectFromSet() returned the handle of the binary semaphore.
//            Take the semaphore now. The semaphore is known to be available so a block time
//            of 0 is used. */
//            xSemaphoreTake( xBinarySemaphore, 0 );
//            /* Whatever processing is necessary when the semaphore is taken can be performed
//            here... */
//        }
//    }
//}

//void vReceiverMultiTask( void *pvParameters )
//{
//    QueueHandle_t xQueueThatContainsData;
//    char *pcReceivedString;
//    /* As per most tasks, this task is implemented within an infinite loop. */
//    for( ;; )
//    {
//        /* Block on the queue set to wait for one of the queues in the set to contain data.
//        Cast the QueueSetMemberHandle_t value returned from xQueueSelectFromSet() to a
//        QueueHandle_t, as it is known all the members of the set are queues (the queue set
//        does not contain any semaphores). */
//        xQueueThatContainsData = ( QueueHandle_t ) xQueueSelectFromSet( xQueueSet,
//        portMAX_DELAY );
//        /* An indefinite block time was used when reading from the queue set, so
//        xQueueSelectFromSet() will not have returned unless one of the queues in the set
//        contained data, and xQueueThatContainsData cannot be NULL. Read from the queue. It
//        is not necessary to specify a block time because it is known the queue contains
//        data. The block time is set to 0. */
//        xQueueReceive( xQueueThatContainsData, &pcReceivedString, 0 );
//        /* Print the string received from the queue. */
//        vPrintString( "%s\r\n", pcReceivedString );
//    }
//}

//int main( void )
//{
//    BSP_Init();
//    /* Create the two queues, both of which send character pointers. The priority
//    of the receiving task is above the priority of the sending tasks, so the queues
//    will never have more than one item in them at any one time*/
//    xCharPointerQueue = xQueueCreate( 1, sizeof( char * ) );
//    xUint32tQueue = xQueueCreate( 1, sizeof( char * ) );
//    xBinarySemaphore = xQueueCreate( 1, sizeof( char * ) );

//    /* Create the queue set. Two queues will be added to the set, each of which can
//    contain 1 item, so the maximum number of queue handles the queue set will ever
//    have to hold at one time is 2 (2 queues multiplied by 1 item per queue). */
//    xQueueSet = xQueueCreateSet( 1 * 3 );//三个队列
//    /* Add the two queues to the set. */
//    xQueueAddToSet( xCharPointerQueue, xQueueSet );
//    xQueueAddToSet( xUint32tQueue, xQueueSet );
//    xQueueAddToSet( xBinarySemaphore, xQueueSet );

//    /* Create the tasks that send to the queues. */
//    xTaskCreate( vSenderTaskChar, "SenderChar", 1000, NULL, 1, NULL );
//    xTaskCreate( vSenderTaskUInt32, "SenderUIint32", 1000, NULL, 1, NULL );
//    xTaskCreate( vSenderTaskBinary, "SenderBinary", 1000, NULL, 1, NULL );

//    /* Create the task that reads from the queue set to determine which of the two
//    queues contain data. */
//    xTaskCreate( vReceiverMultiTask, "Receiver", 1000, NULL, 2, NULL );
//    /* Start the scheduler so the created tasks start executing. */
//    vTaskStartScheduler();
//    /* As normal, vTaskStartScheduler() should not return, so the following lines
//    Will never execute. */
//    for( ;; );
//    return 0;
//}
