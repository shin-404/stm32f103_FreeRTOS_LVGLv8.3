//
// Created by Shin on 2023/4/30.
//

#include "test_functions.h"

/******************************************************************************************/
/* ���� ���� */

#define KEY0_GPIO_PORT                  GPIOE
#define KEY0_GPIO_PIN                   GPIO_PIN_4
#define KEY0_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)   /* PE��ʱ��ʹ�� */

#define KEY1_GPIO_PORT                  GPIOE
#define KEY1_GPIO_PIN                   GPIO_PIN_3
#define KEY1_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)   /* PE��ʱ��ʹ�� */

#define WKUP_GPIO_PORT                  GPIOA
#define WKUP_GPIO_PIN                   GPIO_PIN_0
#define WKUP_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* PA��ʱ��ʹ�� */

/******************************************************************************************/

#define KEY0        HAL_GPIO_ReadPin(KEY0_GPIO_PORT, KEY0_GPIO_PIN)     /* ��ȡKEY0���� */
#define KEY1        HAL_GPIO_ReadPin(KEY1_GPIO_PORT, KEY1_GPIO_PIN)     /* ��ȡKEY1���� */
#define WK_UP       HAL_GPIO_ReadPin(WKUP_GPIO_PORT, WKUP_GPIO_PIN)     /* ��ȡWKUP���� */


#define KEY0_PRES    1              /* KEY0���� */
#define KEY1_PRES    2              /* KEY1���� */
#define WKUP_PRES    3              /* KEY_UP����(��WK_UP) */


/**
 * @brief       ������ʼ������
 * @param       ��
 * @retval      ��
 */
void key_init(void)
{
  GPIO_InitTypeDef gpio_init_struct;
  KEY0_GPIO_CLK_ENABLE();                                     /* KEY0ʱ��ʹ�� */
  KEY1_GPIO_CLK_ENABLE();                                     /* KEY1ʱ��ʹ�� */
  WKUP_GPIO_CLK_ENABLE();                                     /* WKUPʱ��ʹ�� */

  gpio_init_struct.Pin = KEY0_GPIO_PIN;                       /* KEY0���� */
  gpio_init_struct.Mode = GPIO_MODE_INPUT;                    /* ���� */
  gpio_init_struct.Pull = GPIO_PULLUP;                        /* ���� */
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              /* ���� */
  HAL_GPIO_Init(KEY0_GPIO_PORT, &gpio_init_struct);           /* KEY0����ģʽ����,�������� */

  gpio_init_struct.Pin = KEY1_GPIO_PIN;                       /* KEY1���� */
  gpio_init_struct.Mode = GPIO_MODE_INPUT;                    /* ���� */
  gpio_init_struct.Pull = GPIO_PULLUP;                        /* ���� */
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              /* ���� */
  HAL_GPIO_Init(KEY1_GPIO_PORT, &gpio_init_struct);           /* KEY1����ģʽ����,�������� */

  gpio_init_struct.Pin = WKUP_GPIO_PIN;                       /* WKUP���� */
  gpio_init_struct.Mode = GPIO_MODE_INPUT;                    /* ���� */
  gpio_init_struct.Pull = GPIO_PULLDOWN;                      /* ���� */
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              /* ���� */
  HAL_GPIO_Init(WKUP_GPIO_PORT, &gpio_init_struct);           /* WKUP����ģʽ����,�������� */

}


/**
 * @brief       ����ɨ�躯��
 * @note        �ú�������Ӧ���ȼ�(ͬʱ���¶������): WK_UP > KEY1 > KEY0!!
 * @param       mode:0 / 1, ���庬������:
 *   @arg       0,  ��֧��������(���������²���ʱ, ֻ�е�һ�ε��û᷵�ؼ�ֵ,
 *                  �����ɿ��Ժ�, �ٴΰ��²Ż᷵��������ֵ)
 *   @arg       1,  ֧��������(���������²���ʱ, ÿ�ε��øú������᷵�ؼ�ֵ)
 * @retval      ��ֵ, ��������:
 *              KEY0_PRES, 1, KEY0����
 *              KEY1_PRES, 2, KEY1����
 *              WKUP_PRES, 3, WKUP����
 */
uint8_t key_scan(uint8_t mode)
{
  static uint8_t key_up = 1;  /* �������ɿ���־ */
  uint8_t keyval = 0;

  if (mode) key_up = 1;       /* ֧������ */

  if (key_up && (KEY0 == 0 || KEY1 == 0 || WK_UP == 1))  /* �����ɿ���־Ϊ1, ��������һ������������ */
  {
    delay_ms(10);           /* ȥ���� */
    key_up = 0;

    if (KEY0 == 0)  keyval = KEY0_PRES;

    if (KEY1 == 0)  keyval = KEY1_PRES;

    if (WK_UP == 1) keyval = WKUP_PRES;
  }
  else if (KEY0 == 1 && KEY1 == 1 && WK_UP == 0) /* û���κΰ�������, ��ǰ����ɿ� */
  {
    key_up = 1;
  }

  return keyval;              /* ���ؼ�ֵ */
}

/**
 * @brief       �����Ļ�������Ͻ���ʾ"RST"
 * @param       ��
 * @retval      ��
 */
void load_draw_dialog(void)
{
  lcd_clear(WHITE);                                                /* ���� */
  lcd_show_string(lcddev.width - 24, 0, 200, 16, 16, "RST", BLUE); /* ��ʾ�������� */
}

/**
 * @brief       ������
 * @param       x1,y1: �������
 * @param       x2,y2: �յ�����
 * @param       size : ������ϸ�̶�
 * @param       color: �ߵ���ɫ
 * @retval      ��
 */
void lcd_draw_bline(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t size, uint16_t color)
{
  uint16_t t;
  int xerr = 0, yerr = 0, delta_x, delta_y, distance;
  int incx, incy, row, col;

  if (x1 < size || x2 < size || y1 < size || y2 < size)
    return;

  delta_x = x2 - x1; /* ������������ */
  delta_y = y2 - y1;
  row = x1;
  col = y1;

  if (delta_x > 0)
  {
    incx = 1; /* ���õ������� */
  }
  else if (delta_x == 0)
  {
    incx = 0; /* ��ֱ�� */
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
    incy = 0; /* ˮƽ�� */
  }
  else
  {
    incy = -1;
    delta_y = -delta_y;
  }

  if (delta_x > delta_y)
    distance = delta_x; /* ѡȡ�������������� */
  else
    distance = delta_y;

  for (t = 0; t <= distance + 1; t++) /* ������� */
  {
    lcd_fill_circle(row, col, size, color); /* ���� */
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


/* 10�����ص����ɫ(���ݴ�������) */
const uint16_t POINT_COLOR_TBL[10] = {RED, GREEN, BLUE, BROWN, YELLOW, MAGENTA, CYAN, LIGHTBLUE, BRRED, GRAY};

/**
 * @brief       ���ݴ��������Ժ���
 * @param       ��
 * @retval      ��
 */
void ctp_test(void)
{
  uint8_t t = 0;
  uint8_t i = 0;
  uint16_t lastpos[10][2];        /* ���һ�ε����� */
  uint8_t maxp = 5;

  if (lcddev.id == 0X1018)maxp = 10;

  while (1)
  {
    tp_dev.scan(0);

    for (t = 0; t < maxp; t++)
    {
      if ((tp_dev.sta) & (1 << t))
      {
        if (tp_dev.x[t] < lcddev.width && tp_dev.y[t] < lcddev.height)  /* ��������Ļ��Χ�� */
        {
          if (lastpos[t][0] == 0XFFFF)
          {
            lastpos[t][0] = tp_dev.x[t];
            lastpos[t][1] = tp_dev.y[t];
          }

          lcd_draw_bline(lastpos[t][0], lastpos[t][1], tp_dev.x[t], tp_dev.y[t], 2, POINT_COLOR_TBL[t]); /* ���� */
          lastpos[t][0] = tp_dev.x[t];
          lastpos[t][1] = tp_dev.y[t];

          if (tp_dev.x[t] > (lcddev.width - 24) && tp_dev.y[t] < 20)
          {
            load_draw_dialog();/* ��� */
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
 * @brief       ���败�������Ժ���
 * @param       ��
 * @retval      ��
 */
void rtp_test(void)
{
  uint8_t key;
  uint8_t i = 0;

  while (1)
  {
    key = key_scan(0);
    tp_dev.scan(0);

    if (tp_dev.sta & TP_PRES_DOWN)  /* ������������ */
    {
      if (tp_dev.x[0] < lcddev.width && tp_dev.y[0] < lcddev.height)
      {
        if (tp_dev.x[0] > (lcddev.width - 24) && tp_dev.y[0] < 16)
        {
          load_draw_dialog(); /* ��� */
        }
        else
        {
          tp_draw_big_point(tp_dev.x[0], tp_dev.y[0], RED);   /* ���� */
        }
      }
    }
    else
    {
      delay_ms(10);       /* û�а������µ�ʱ�� */
    }

    if (key == KEY0_PRES)   /* KEY0����,��ִ��У׼���� */
    {
      lcd_clear(WHITE);   /* ���� */
      tp_adjust();        /* ��ĻУ׼ */
      tp_save_adjust_data();
      load_draw_dialog();
    }

    i++;

    if (i % 20 == 0)HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
  }
}