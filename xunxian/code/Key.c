#include "Key.h"

Key_t key;

uint8 Key_Scan(uint8* State)
{
	key.val = 0;
	key.up = 0;
	key.down = 0;
	key.state = 0;
	
	if (gpio_get_level(KEY1_PIN) == GPIO_LOW)
	{
		key.val = 1;
	}
	if (gpio_get_level(KEY2_PIN) == GPIO_LOW)
	{
		key.val = 2;
	}
	if (gpio_get_level(KEY3_PIN) == GPIO_LOW)
	{
		key.val = 3;
	}
	if (gpio_get_level(KEY4_PIN) == GPIO_LOW)
	{
		key.val = 4;
	}
	
	key.down = key.val & (key.val ^key.old);
	key.up = ~key.val & (key.val ^ key.old);
	key.old = key.val;
	
	if (key.down)
	{
		key.state = KEY_STATE_DOWN;
		key.delay = 0;
	}
	
	else if (key.up)
	{
		key.val = key.up;
		if (key.delay < KEY_TIM_LONG)
		{
			key.state = KEY_STATE_PRESSED; // 小于1秒松手，正常触发短按
		}
		else
		{
			// 如果按的时间超过了长按阈值，松手时就不应该再触发任何状态了
			key.state = 0; 
		}
	}
	else // 在按住不放的期间
	{
		if (key.val)
		{
			if (key.delay > KEY_TIM_LONG)
			{
				// 只要按住时间一超过 1000ms，立刻、持续地触发长按状态
				key.state = KEY_STATE_LONG;
			}
			else if (key.delay > KEY_TIM_PRESSING)
			{
				// 在 300ms 到 1000ms 之间，触发“持续按压”状态（可选）
				key.state = KEY_STATE_PERESSING;
			}
		}
	}
	
	*State = key.state;
	return key.val;
}

void Key_Proc (uint8* Key,uint8* Data)
{
	uint8 KeyState;
	switch (Key_Scan(&KeyState))
	{
		case 1:
		{
			if (KeyState == KEY_STATE_PRESSED)
			{
				*Key = 1;
				Data[0] =!Data[0];
                Data[1] = 1;
			}
			if (KeyState == KEY_STATE_LONG)
			{
				*Key = 2;
			}
			if (KeyState == KEY_STATE_PERESSING)
			{
				*Key = 3;
			}
			break;
		}
		case 2:
		{
			if (KeyState == KEY_STATE_PRESSED)
			{
				*Key = 4;
			}
			break;
		}
		case 3:
		{
			if (KeyState == KEY_STATE_PRESSED)
			{
				*Key = 5;
			}
			break;
		}
		case 4:
		{
			
			break;
		}
		default:break;
	}
}

void pit_handler_key (void)
{
    key.delay++;
}