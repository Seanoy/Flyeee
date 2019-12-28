//#include "rtos_test.h"
////�ܵ���˵�����demo��ʵ�ַ�����ͨ������ָ�����ﵽ��������ݵ�Ŀ��

///* Declare a variable of type QueueHandle_t to hold the handle of the queue being created. */
//QueueHandle_t xPointerQueue;
///* Create a queue that can hold a maximum of 5 pointers, in this case character pointers. */
//xPointerQueue = xQueueCreate( 5, sizeof( char * ) );

///* A task that obtains a buffer, writes a string to the buffer, then sends the address of the
//buffer to the queue created in Listing 52. */
////ʹ�ö��з���һ��ָ�뵽buffer
//void vStringSendingTask( void *pvParameters )
//{
//    char *pcStringToSend;
//    const size_t xMaxStringLength = 50;
//    BaseType_t xStringNumber = 0;
//    for( ;; )
//    {
//        /* Obtain a buffer that is at least xMaxStringLength characters big. The implementation
//        of prvGetBuffer() is not shown �C it might obtain the buffer from a pool of pre-allocated
//        buffers, or just allocate the buffer dynamically. */
//        pcStringToSend = ( char * ) prvGetBuffer( xMaxStringLength );
//        /* Write a string into the buffer. */
//        snprintf( pcStringToSend, xMaxStringLength, "String number %d\r\n", xStringNumber );
//        /* Increment the counter so the string is different on each iteration of this task. */
//        xStringNumber++;
//        /* Send the address of the buffer to the queue that was created in Listing 52. The
//        address of the buffer is stored in the pcStringToSend variable.*/
//        xQueueSend( xPointerQueue, /* The handle of the queue. */
//        &pcStringToSend, /* The address of the pointer that points to the buffer. */
//        portMAX_DELAY );
//    }
//}


///* A task that receives the address of a buffer from the queue created in Listing 52, and
//written to in Listing 53. The buffer contains a string, which is printed out. */
////ʹ�ö��д�buffer����һ��ָ��
//void vStringReceivingTask( void *pvParameters )
//{
//    char *pcReceivedString;
//    for( ;; )
//    {
//        /* Receive the address of a buffer. */
//        xQueueReceive( xPointerQueue, /* The handle of the queue. */
//        &pcReceivedString, /* Store the buffer��s address in pcReceivedString. */
//        portMAX_DELAY );
//        /* The buffer holds a string, print it out. */
//        vPrintString( "%s\r\n",pcReceivedString );
//        /* The buffer is not required any more - release it so it can be freed, or re-used. */
//        prvReleaseBuffer( pcReceivedString );
//    }
//}

