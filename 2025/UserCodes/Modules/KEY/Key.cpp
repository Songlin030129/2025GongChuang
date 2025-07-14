#include "Key.h"
KEY key1, key2, key3;
KEYS keys;

/**
 * @brief 按键按下回调函数
 *
 */
__attribute__((weak)) void KEY_KeyClickCallback(KEY* key)
{
    UNUSED(key);
}
/**
 * @brief 按键多击回调函数
 *
 */
__attribute__((weak)) void KEY_MultipleClickCallback(KEY* key)
{
    UNUSED(key);
}
/**
 * @brief 按键长按回调函数
 *
 */
__attribute__((weak)) void KEY_LongHoldCallback(KEY* key)
{
    UNUSED(key);
}
/**
 * @brief 按键长按触发回调函数
 *
 */
__attribute__((weak)) void KEY_HoldTriggerCallback(KEY* key)
{
    UNUSED(key);
}
/**
 * @brief 添加按键
 *
 * @param key 按键对象
 * @param GPIOx GPIO端口
 * @param GPIO_Pin GPIO引脚
 */
void KEYS::AddKey(KEY* key, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    key->GPIOx = GPIOx;
    key->GPIO_Pin = GPIO_Pin;
    key->state = Release;
    keys.push_back(key);
    keyCnt++;
}
/**
 * @brief 按键扫描处理状态机
 *
 */
void KEYS::KeysHandler()
{
    // 获取时间戳
    nowTime = HAL_GetTick();
    deltaTime = nowTime - lastTime;
    lastTime = nowTime;
    for (int i = 0; i < keyCnt; i++)
    {
        // 键值同步
        keys[i]->val = !HAL_GPIO_ReadPin(keys[i]->GPIOx, keys[i]->GPIO_Pin);
        keys[i]->preVal = keys[i]->val;

        // 按下计时
        if (!keys[i]->val)
        {
            if (keys[i]->state == LongHold)
            {
                keys[i]->holdTime = keys[i]->pressTimer;
            }
            keys[i]->pressTimer = 0;
        }
        if (keys[i]->preVal & keys[i]->val)
        {
            keys[i]->pressTimer += deltaTime;
        }
        // 间隔计时
        if (keys[i]->state == MultiClick)
        {
            keys[i]->intervalTimer += deltaTime;
        }
        else
        {
            keys[i]->intervalTimer = 0;
        }

        // 事件生成
        switch (keys[i]->state)
        {
        case Release:
            keys[i]->clickCnt = 0;

            if (keys[i]->val)
            {
                keys[i]->state = PrePress;
            }
            break;
        case PrePress:

            if (!keys[i]->val)
            {
                keys[i]->state = Release;
            }
            else if (keys[i]->pressTimer >= KEY_ClickThreshold)
            {
                keys[i]->state = Prelong;
            }
            break;
        case Prelong:

            if (!keys[i]->val)
            {
                keys[i]->state = MultiClick;
                keys[i]->clickCnt++;
            }
            else if (keys[i]->pressTimer >= KEY_HoldThreshold)
            {
                keys[i]->state = LongHold;
                keys[i]->triggerTimer = KEY_HoldTriggerFirstThreshold;
                KEY_LongHoldCallback(keys[i]);
            }
            break;
        case LongHold:

            if (keys[i]->triggerTimer > 0)
                keys[i]->triggerTimer -= deltaTime;
            else
            {
                keys[i]->triggerTimer = KEY_HoldTriggerThreshold;
                KEY_HoldTriggerCallback(keys[i]);
            }

            if (!keys[i]->val)
            {
                keys[i]->state = Release;
            }

            break;
        case MultiClick:

            if (keys[i]->intervalTimer >= KEY_IntervalThreshold)
            {
                if (keys[i]->clickCnt > 1)
                {
                    KEY_MultipleClickCallback(keys[i]);
                }
                else if (keys[i]->clickCnt == 1)
                {
                    KEY_KeyClickCallback(keys[i]);
                }
                keys[i]->state = Release;
            }
            else if (keys[i]->pressTimer >= KEY_ClickThreshold)
            {
                keys[i]->state = Prelong;
            }
            break;

        default:
            break;
        }
    }
}
