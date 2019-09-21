/**
******************************************************************************
* @brief   调试辅助宏等
* @details None
* @file    custom_debug.h
* @author  GYC
* @version V1.0

* @date    2017-11-23
* @note    注解
******************************************************************************
* @attention None
* @warning None
* @enum  	引用了某个枚举，Doxygen会在该枚举处产生一个链接. eg：@enum CTest::MyEnum
* @var      引用了某个变量，Doxygen会在该枚举处产生一个链接. eg：@var CTest::m_FileKey
******************************************************************************

======================================================================================
Revision History:
					  Nodification
	          Author			     Data		                       Major Changes
          --------------      -------------------         --------------------------------------
* @since	   GYC					2017.11.23								creat
          ======================================================================================
*              GYC                  2017.12.18                          增加二进制打印
*              GYC                  2017.12.20                          修改DEBUG_MSG(增加__func__)

*/


/**
 * @brief     简介
 * @see       一段包含其他部分引用的注释，中间包含对其他代码项的名称，自动产生对其的引用链接。
 * @param     参数描述 eg: cnt: 计数
 * @return    描述返回值情况 eg: @return 本函数返回执行结果，若成功则返回TRUE，否则返回FLASE
 * @retval    描述返回值类型 eg: @retval NULL 空字符串 @retval !NULL 非空字符串。
 * @exception 可能产生的异常描述 eg: @exception 本函数执行可能会产生超出范围的异常
 * @enum  	  引用了某个枚举，Doxygen会在该枚举处产生一个链接. eg：@enum CTest::MyEnum
 * @var       引用了某个变量，Doxygen会在该枚举处产生一个链接. eg：@var CTest::m_FileKey
 * @attention None
 * @warning None
 * @since 通常用来说明从什么版本、时间写此部分代码
 * @todo 对将要做的事情进行注释
 * @pre  用来说明代码项的前提条件。
 * @post 用来说明代码项之后的使用条件。
 * @deprecated 这个函数可能会在将来的版本中取消
 * @par 示例: 
   @code
    // @code  在注释中开始说明一段代码，直到@endcode命令。
    extern  IThread *pThread;
    HANDLE hEvent = pThread->GetEventHandle();
    while(WaitForSingleObject(hEvent,0) != WAIT_OBJECT_0)
    {
    //Do something
    }
    @endcode
 */
 
/**    颜色的枚举定义  
  *   
  *    该枚举定义了系统中需要用到的颜色\n  
  *    可以使用该枚举作为系统中颜色的标识  
  */ 

  
  
#ifndef __CUSTOM_DEBUG_H
#define __CUSTOM_DEBUG_H

#include <string.h>
//example: printf("filename:%s\n", filename(__FILE__));		//为了去掉路径
#define filename(x) strrchr(x,'\\')?strrchr(x,'\\')+1:x

#ifdef CUSTOM_DEBUG_ON
#define DEBUG_MSG(fmt,arg...)	printf("<<DMSG>@File:%s@Line:%d@FUNC:%s>"fmt"\n",filename(__FILE__),__LINE__,__func__,##arg)
#else
#define DEBUG_MSG(fmt,arg...)    
#endif


//打印数组(包含数组名) 
#define PRINT_ARRAY(array, length) do {         \
    for (int i = 0; i<length; i++) {            \
        printf(#array);	                        \
        printf("[%d]:%d\n", i, array[i]);       \
    }                                           \
}while(0);

//将一个整数以二进制方式打印出来
#define PRINT_BINARY(num) do {                  \
    printf(#num);                               \
    printf("_L:");                              \
    for (uint32_t i = num; i>0; i /= 2) {       \
        printf("%d", i%2);                      \
    }                                           \
    printf("\r\n");                             \
}while(0);

#endif

