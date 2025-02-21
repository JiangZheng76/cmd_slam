#ifndef __SYLAR_NONCOPYABLE_H__
#define __SYLAR_NONCOPYABLE_H__

/**
 * @brief 增加一个不可继承类
 * @description: 
 * @return {*}
 */
class Noncopyable{
public:
    Noncopyable(){
    }
    ~Noncopyable(){
    }
private:
Noncopyable(Noncopyable& n) = delete;

Noncopyable& operator=(Noncopyable& n ) = delete;

};

#endif