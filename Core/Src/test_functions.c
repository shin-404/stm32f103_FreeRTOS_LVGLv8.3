//
// Created by Shin on 2023/4/30.
//

#include "test_functions.h"


/**
 * @brief       按键初始化函数
 * @param       无
 * @retval      无
 */
void key_init(void)
{
  GPIO_InitTypeDef gpio_init_struct;
  KEY0_GPIO_CLK_ENABLE();                                     /* KEY0时钟使能 */
  KEY1_GPIO_CLK_ENABLE();                                     /* KEY1时钟使能 */
  WKUP_GPIO_CLK_ENABLE();                                     /* WKUP时钟使能 */

  gpio_init_struct.Pin = KEY0_GPIO_PIN;                       /* KEY0引脚 */
  gpio_init_struct.Mode = GPIO_MODE_INPUT;                    /* 输入 */
  gpio_init_struct.Pull = GPIO_PULLUP;                        /* 上拉 */
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              /* 高速 */
  HAL_GPIO_Init(KEY0_GPIO_PORT, &gpio_init_struct);           /* KEY0引脚模式设置,上拉输入 */

  gpio_init_struct.Pin = KEY1_GPIO_PIN;                       /* KEY1引脚 */
  gpio_init_struct.Mode = GPIO_MODE_INPUT;                    /* 输入 */
  gpio_init_struct.Pull = GPIO_PULLUP;                        /* 上拉 */
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              /* 高速 */
  HAL_GPIO_Init(KEY1_GPIO_PORT, &gpio_init_struct);           /* KEY1引脚模式设置,上拉输入 */

  gpio_init_struct.Pin = WKUP_GPIO_PIN;                       /* WKUP引脚 */
  gpio_init_struct.Mode = GPIO_MODE_INPUT;                    /* 输入 */
  gpio_init_struct.Pull = GPIO_PULLDOWN;                      /* 下拉 */
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              /* 高速 */
  HAL_GPIO_Init(WKUP_GPIO_PORT, &gpio_init_struct);           /* WKUP引脚模式设置,下拉输入 */

}


/**
 * @brief       按键扫描函数
 * @note        该函数有响应优先级(同时按下多个按键): WK_UP > KEY1 > KEY0!!
 * @param       mode:0 / 1, 具体含义如下:
 *   @arg       0,  不支持连续按(当按键按下不放时, 只有第一次调用会返回键值,
 *                  必须松开以后, 再次按下才会返回其他键值)
 *   @arg       1,  支持连续按(当按键按下不放时, 每次调用该函数都会返回键值)
 * @retval      键值, 定义如下:
 *              KEY0_PRES, 1, KEY0按下
 *              KEY1_PRES, 2, KEY1按下
 *              WKUP_PRES, 3, WKUP按下
 */
uint8_t key_scan(uint8_t mode)
{
  static uint8_t key_up = 1;  /* 按键按松开标志 */
  uint8_t keyval = 0;

  if (mode) key_up = 1;       /* 支持连按 */

  if (key_up && (KEY0 == 0 || KEY1 == 0 || WK_UP == 1))  /* 按键松开标志为1, 且有任意一个按键按下了 */
  {
    delay_ms(10);           /* 去抖动 */
    key_up = 0;

    if (KEY0 == 0)  keyval = KEY0_PRES;

    if (KEY1 == 0)  keyval = KEY1_PRES;

    if (WK_UP == 1) keyval = WKUP_PRES;
  }
  else if (KEY0 == 1 && KEY1 == 1 && WK_UP == 0) /* 没有任何按键按下, 标记按键松开 */
  {
    key_up = 1;
  }

  return keyval;              /* 返回键值 */
}

/**
 * @brief       清空屏幕并在右上角显示"RST"
 * @param       无
 * @retval      无
 */
void load_draw_dialog(void)
{
  lcd_clear(WHITE);                                                /* 清屏 */
  lcd_show_string(lcddev.width - 24, 0, 200, 16, 16, "RST", BLUE); /* 显示清屏区域 */
}

/**
 * @brief       画粗线
 * @param       x1,y1: 起点坐标
 * @param       x2,y2: 终点坐标
 * @param       size : 线条粗细程度
 * @param       color: 线的颜色
 * @retval      无
 */
void lcd_draw_bline(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t size, uint16_t color)
{
  uint16_t t;
  int xerr = 0, yerr = 0, delta_x, delta_y, distance;
  int incx, incy, row, col;

  if (x1 < size || x2 < size || y1 < size || y2 < size)
    return;

  delta_x = x2 - x1; /* 计算坐标增量 */
  delta_y = y2 - y1;
  row = x1;
  col = y1;

  if (delta_x > 0)
  {
    incx = 1; /* 设置单步方向 */
  }
  else if (delta_x == 0)
  {
    incx = 0; /* 垂直线 */
  }
  else
  {
    incx = -1;
    delta_x = -delta_x;
  }

  if (delta_y > 0)
  {
    incy = 1;
  }
  else if (delta_y == 0)
  {
    incy = 0; /* 水平线 */
  }
  else
  {
    incy = -1;
    delta_y = -delta_y;
  }

  if (delta_x > delta_y)
    distance = delta_x; /* 选取基本增量坐标轴 */
  else
    distance = delta_y;

  for (t = 0; t <= distance + 1; t++) /* 画线输出 */
  {
    lcd_fill_circle(row, col, size, color); /* 画点 */
    xerr += delta_x;
    yerr += delta_y;

    if (xerr > distance)
    {
      xerr -= distance;
      row += incx;
    }

    if (yerr > distance)
    {
      yerr -= distance;
      col += incy;
    }
  }
}


/* 10个触控点的颜色(电容触摸屏用) */
const uint16_t POINT_COLOR_TBL[10] = {RED, GREEN, BLUE, BROWN, YELLOW, MAGENTA, CYAN, LIGHTBLUE, BRRED, GRAY};

/**
 * @brief       电容触摸屏测试函数
 * @param       无
 * @retval      无
 */
void ctp_test(void)
{
  uint8_t t = 0;
  uint8_t i = 0;
  uint16_t lastpos[10][2];        /* 最后一次的数据 */
  uint8_t maxp = 5;

  if (lcddev.id == 0X1018)maxp = 10;

  while (1)
  {
    tp_dev.scan(0);

    for (t = 0; t < maxp; t++)
    {
      if ((tp_dev.sta) & (1 << t))
      {
        if (tp_dev.x[t] < lcddev.width && tp_dev.y[t] < lcddev.height)  /* 坐标在屏幕范围内 */
        {
          if (lastpos[t][0] == 0XFFFF)
          {
            lastpos[t][0] = tp_dev.x[t];
            lastpos[t][1] = tp_dev.y[t];
          }

          lcd_draw_bline(lastpos[t][0], lastpos[t][1], tp_dev.x[t], tp_dev.y[t], 2, POINT_COLOR_TBL[t]); /* 画线 */
          lastpos[t][0] = tp_dev.x[t];
          lastpos[t][1] = tp_dev.y[t];

          if (tp_dev.x[t] > (lcddev.width - 24) && tp_dev.y[t] < 20)
          {
            load_draw_dialog();/* 清除 */
          }
        }
      }
      else
      {
        lastpos[t][0] = 0XFFFF;
      }
    }

    delay_ms(5);
    i++;

    if (i % 20 == 0)HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
  }
}


/**
 * @brief       电阻触摸屏测试函数
 * @param       无
 * @retval      无
 */
void rtp_test(void)
{
  uint8_t key;
  uint8_t i = 0;

  while (1)
  {
    key = key_scan(0);
    tp_dev.scan(0);

    if (tp_dev.sta & TP_PRES_DOWN)  /* 触摸屏被按下 */
    {
      if (tp_dev.x[0] < lcddev.width && tp_dev.y[0] < lcddev.height)
      {
        if (tp_dev.x[0] > (lcddev.width - 24) && tp_dev.y[0] < 16)
        {
          load_draw_dialog(); /* 清除 */
        }
        else
        {
          tp_draw_big_point(tp_dev.x[0], tp_dev.y[0], RED);   /* 画点 */
        }
      }
    }
    else
    {
      delay_ms(10);       /* 没有按键按下的时候 */
    }

    if (key == KEY0_PRES)   /* KEY0按下,则执行校准程序 */
    {
      lcd_clear(WHITE);   /* 清屏 */
      tp_adjust();        /* 屏幕校准 */
      tp_save_adjust_data();
      load_draw_dialog();
    }

    i++;

    if (i % 20 == 0)HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
  }
}

void tftLCD_test(void)
{

  key_init();
  lcd_init();
  tp_dev.init();
  lcd_show_string(30, 50, 200, 16, 16, "STM32", RED);
  lcd_show_string(30, 70, 200, 16, 16, "TOUCH TEST", RED);
  lcd_show_string(30, 90, 200, 16, 16, "ATOM@ALIENTEK", RED);

  if (tp_dev.touchtype != 0XFF)
  {
    lcd_show_string(30, 110, 200, 16, 16, "Press KEY0 to Adjust", RED); /* 电阻屏才显示 */
  }

  delay_ms(1500);
  load_draw_dialog();

  if (tp_dev.touchtype & 0X80)
  {
    ctp_test(); /* 电容屏测试 */
  }
  else
  {
    rtp_test(); /* 电阻屏测试 */
  }
}
