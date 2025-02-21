/*
 * @Author: Jiangzheng 2440877322@qq.com
 * @Date: 2024-02-24 14:25:12
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-02-26 18:18:49
 * @FilePath: /mysylar/mytest/test_iomanager.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "sylar.h"
#include "iomanager.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>



void readsocket(){
    // int fd = socket(AF_INET,SOCK_STREAM,0);
    // fcntl(fd,F_SETFL,O_NONBLOCK);

    // sockaddr_in addr;
    // memset(&addr, 0,sizeof(addr));
    // addr.sin_family = AF_INET;
    // addr.sin_port = htons(80);
    // inet_pton(AF_INET,"157.148.69.80",&addr.sin_addr.s_addr);
    // int rt3 = connect(fd,(const sockaddr*)&addr,sizeof(addr));
    
    int m_pipe[2];
    int rt3 = pipe(m_pipe);
    fcntl(m_pipe[0],F_SETFL,O_NONBLOCK);
    fcntl(m_pipe[1],F_SETFL,O_NONBLOCK);
    SYLAR_ASSERT(!rt3);
    char str[64] = "write pipe.";
    
    mysylar::IOManager::GetThis()->addEvent(m_pipe[0],mysylar::IOManager::READ,[](){
        SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << "m_pipe[0] read.";
    });
    mysylar::IOManager::GetThis()->addEvent(m_pipe[0],mysylar::IOManager::WRITE,[](){
        SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << "m_pipe[0] write.";
    });

    mysylar::IOManager::GetThis()->addEvent(m_pipe[1],mysylar::IOManager::WRITE,[&](){
        SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << "m_pipe[1] write callback";
        char res[64];
        read(m_pipe[0],res,64);
        close(m_pipe[0]);
        close(m_pipe[1]);
    });

    mysylar::IOManager::GetThis()->addEvent(m_pipe[1],mysylar::IOManager::READ,[&](){
        SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << "m_pipe[1] read callback";
    });
    
    write(m_pipe[1],str,64);
    // std::cout << res << rt5 << std::endl;
}
void test_fiber(){
    mysylar::IOManager iom(3,true,"iom");
    iom.schedule(&readsocket);
    sleep(1);
}
mysylar::Timer::ptr s_timer1;
mysylar::Timer::ptr s_timer2;
mysylar::Timer::ptr s_timer3;
void test_timer(){
    mysylar::IOManager iom(1,false);
    s_timer1 = iom.addTimer(500,[](){
        SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << "500ms times up.";
    },true);
    s_timer2 = iom.addTimer(100,[](){
        SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << "100ms times up.";
    },true);
    s_timer3 = iom.addTimer(1000,[](){
        static int times = 0;
        SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << times << "s end.";
        if(times < 1 ){
            times++;
        }else {
            s_timer1->cancel();
            s_timer2->cancel();
            s_timer3->cancel();
            // mysylar::IOManager::GetThis()->stop();
        }
    },true);
}
int main(){
    // test_fiber();
    test_timer();
    
    return 0;
}