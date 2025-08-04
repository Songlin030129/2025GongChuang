#ifndef __KEY_H__
#define __KEY_H__

#include "common_inc.h"
#include "vector"
#define KEY_ClickThreshold 20             // 按键按下时间阈值
#define KEY_HoldThreshold 600             // 按键长按时间阈值
#define KEY_IntervalThreshold 200         // 按键松开后再次按下时间间隔阈值
#define KEY_HoldTriggerFirstThreshold 600 // 按键长按时间阈值
#define KEY_HoldTriggerThreshold 100      // 按键长按持续触发时间间隔
typedef enum
{
    Release = 0,
    PrePress,
    Prelong,
    LongHold,
    MultiClick
} states; // 状态枚举
class KEY
{
public:
    GPIO_TypeDef* GPIOx;
    uint16_t GPIO_Pin;

    states state;
    uint8_t preVal = 1;
    uint8_t val = 1;
    uint32_t pressTimer;    // 按下计时
    uint32_t intervalTimer; // 放开计时
    int16_t triggerTimer;   // 长按触发计时
    uint32_t holdTime;      // 长按计时
    uint8_t clickCnt;       // 按下计数

private:
};

class KEYS
{
public:
    /**
     * @brief 添加按键
     *
     * @param key 按键对象
     * @param GPIOx GPIO端口
     * @param GPIO_Pin GPIO引脚
     */
    void AddKey(KEY* key, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
    /**
     * @brief 按键扫描处理状态机
     *
     */
    void KeysHandler();

private:
    std::vector<KEY*> keys;
    uint8_t keyCnt = 0;
    uint32_t nowTime, lastTime, deltaTime;
};

/**
 * @brief 按键按下回调函数
 *
 */
void KEY_KeyClickCallback(KEY* key);
/**
 * @brief 按键多击回调函数
 *
 */
void KEY_MultipleClickCallback(KEY* key);
/**
 * @brief 按键长按回调函数
 *
 */
void KEY_LongHoldCallback(KEY* key);
/**
 * @brief 按键长按触发回调函数
 *
 */
void KEY_HoldTriggerCallback(KEY* key);

extern KEY key1, key2, key3, key4;
extern KEYS keys;


#endif // __KEY_H__