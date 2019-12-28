//#include "rtos_test.h"

//TimerHandle_t xOneShotTimer, xAutoReloadTimer;
//定时器计时结束进入回调函数后，先被读取ID，然后重设ID值为原本值加一，当ID值达到5的时候关闭定时器
//static void prvTimerCallback( TimerHandle_t xTimer )
//{
//    TickType_t xTimeNow;
//    uint32_t ulExecutionCount;
//    /* A count of the number of times this software timer has expired is stored in the timer's
//    ID. Obtain the ID, increment it, then save it as the new ID value. The ID is a void
//    pointer, so is cast to a uint32_t. */
//    ulExecutionCount = ( uint32_t ) pvTimerGetTimerID( xTimer );
//    ulExecutionCount++;
//    vTimerSetTimerID( xTimer, ( void * ) ulExecutionCount );
//    /* Obtain the current tick count. */
//    xTimeNow = xTaskGetTickCount();
//    /* The handle of the one-shot timer was stored in xOneShotTimer when the timer was created.
//    Compare the handle passed into this function with xOneShotTimer to determine if it was the
//    one-shot or auto-reload timer that expired, then output a string to show the time at which
//    the callback was executed. */
//    if( xTimer == xOneShotTimer )
//    {
//        vPrintStringAndNumber( "One-shot timer callback executing :%d", xTimeNow );
//    }
//    else
//    {
//        /* xTimer did not equal xOneShotTimer, so it must have been the auto-reload timer that
//        expired. */
//        vPrintStringAndNumber( "Auto-reload timer callback executing :%d", xTimeNow );
//        if( ulExecutionCount == 5 )
//        {
//            /* Stop the auto-reload timer after it has executed 5 times. This callback function
//            executes in the context of the RTOS daemon task so must not call any functions that
//            might place the daemon task into the Blocked state. Therefore a block time of 0 is
//            used. */
//            xTimerStop( xTimer, 0 );
//        }
//    }
//}



///* The periods assigned to the one-shot and auto-reload timers are 3.333 second and half a
//second respectively. */
//#define mainONE_SHOT_TIMER_PERIOD  pdMS_TO_TICKS( 3333 )
//#define mainAUTO_RELOAD_TIMER_PERIOD pdMS_TO_TICKS( 500 )
//int main( void )
//{
//    BaseType_t xTimer1Started, xTimer2Started;
//    BSP_Init();
//    /* Create the one shot timer, storing the handle to the created timer in xOneShotTimer. */
//    /* Create the one shot timer software timer, storing the handle in xOneShotTimer. */
//    xOneShotTimer = xTimerCreate( "OneShot",
//    mainONE_SHOT_TIMER_PERIOD,
//    pdFALSE,
//    /* The timer’s ID is initialized to 0. */
//    0,
//    /* prvTimerCallback() is used by both timers. */
//    prvTimerCallback );
//    /* Create the auto-reload software timer, storing the handle in xAutoReloadTimer */
//    xAutoReloadTimer = xTimerCreate( "AutoReload",
//    mainAUTO_RELOAD_TIMER_PERIOD,
//    pdTRUE,
//    /* The timer’s ID is initialized to 0. */
//    0,
//    /* prvTimerCallback() is used by both timers. */
//    prvTimerCallback );
//    /* Check the software timers were created. */
//    if( ( xOneShotTimer != NULL ) && ( xAutoReloadTimer != NULL ) )
//    {
//        /* Start the software timers, using a block time of 0 (no block time). The scheduler has
//        not been started yet so any block time specified here would be ignored anyway. */
//        xTimer1Started = xTimerStart( xOneShotTimer, 0 );
//        xTimer2Started = xTimerStart( xAutoReloadTimer, 0 );
//        /* The implementation of xTimerStart() uses the timer command queue, and xTimerStart()
//        will fail if the timer command queue gets full. The timer service task does not get
//        created until the scheduler is started, so all commands sent to the command queue will
//        stay in the queue until after the scheduler has been started. Check both calls to
//        xTimerStart() passed. */
//        if( ( xTimer1Started == pdPASS ) && ( xTimer2Started == pdPASS ) )
//        {
//            /* Start the scheduler. */
//            vTaskStartScheduler();
//        }
//    }
//    /* As always, this line should not be reached. */
//    for( ;; );
//}
