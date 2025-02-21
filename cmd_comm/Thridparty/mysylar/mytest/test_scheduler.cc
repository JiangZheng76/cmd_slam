/*
 * @Author: Jiangzheng 2440877322@qq.com
 * @Date: 2024-02-20 15:29:56
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-02-23 13:37:54
 * @FilePath: /mysylar/mytest/test_scheduler.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "sylar.h"
#include "scheduler.h"

void test_scheduler(){
    static int i=0;
    SYLAR_LOG_INFO(SYLAR_LOG_ROOT())<< "test scheduler." << i++ << " : " << mysylar::GetThreadId();
}
int main(){
    mysylar::Scheduler sch(5,true,"sch");
    sch.start();
    for(int i=0 ;i<100;i++){
        mysylar::Fiber::ptr fiber(new mysylar::Fiber(&test_scheduler));
        sch.schedule(fiber);
    }
    sleep(3);
    sch.stop();
    return 0;
}
