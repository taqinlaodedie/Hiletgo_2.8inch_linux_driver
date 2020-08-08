#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/list.h>
#include <linux/platform_data/dma-bcm2708.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/printk.h>
#include <linux/console.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/cred.h>
#include <linux/gpio.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>

/* 控制TFT屏幕的GPIO */
#define DC_IO	17
#define CS_IO	27
#define RST_IO	22
#define LED_IO	10

/* SPI register offsets */
#define SPI_CS			0x00
#define SPI_FIFO		0x04
#define SPI_CLK			0x08
#define SPI_DLEN		0x0c
#define SPI_LTOH		0x10
#define SPI_DC			0x14

/* 物理地址 */
#define GPIO_BASE		0x3f200000
#define SPI_BASE		0x3f204000

/* Bitfields in CS */
#define SPI_CS_LEN_LONG	0x02000000
#define SPI_CS_DMA_LEN	0x01000000
#define SPI_CS_CSPOL2	0x00800000
#define SPI_CS_CSPOL1	0x00400000
#define SPI_CS_CSPOL0	0x00200000
#define SPI_CS_RXF		0x00100000
#define SPI_CS_RXR		0x00080000
#define SPI_CS_TXD		0x00040000
#define SPI_CS_RXD		0x00020000
#define SPI_CS_DONE		0x00010000
#define SPI_CS_LEN		0x00002000
#define SPI_CS_REN		0x00001000
#define SPI_CS_ADCS		0x00000800
#define SPI_CS_INTR		0x00000400
#define SPI_CS_INTD		0x00000200
#define SPI_CS_DMAEN	0x00000100
#define SPI_CS_TA		0x00000080
#define SPI_CS_CSPOL	0x00000040
#define SPI_CS_CLEAR_RX	0x00000020
#define SPI_CS_CLEAR_TX	0x00000010
#define SPI_CS_CPOL		0x00000008
#define SPI_CS_CPHA		0x00000004
#define SPI_CS_CS_10	0x00000002
#define SPI_CS_CS_01	0x00000001
#define SPI_TIMEOUT_MS	150

#define X 240
#define Y 320

typedef struct spi_regs_struct {
    unsigned spi_cs;
    unsigned spi_fifo;
    unsigned spi_clk;
	unsigned spi_dlen;
	unsigned spi_ltoh;
	unsigned spi_dc;
} spi_regs_t;

static void tft_lcdfb_imageblit(struct fb_info *info, const struct fb_image *image);
static int tft_lcdfb_setcolreg(unsigned int regno,unsigned int red,unsigned int green,unsigned int blue,unsigned int transp,struct fb_info *info);

static struct fb_ops tft_lcdfb_ops = {
	.owner			= THIS_MODULE,
	.fb_setcolreg	= tft_lcdfb_setcolreg,
	.fb_imageblit	= tft_lcdfb_imageblit
};

static struct fb_info *tft_lcd;/*分配info结构体*/

static volatile spi_regs_t *spi_regs;

// SPI发送一个字节
static void spi_write_byte(u8 data)
{
	unsigned val;
	
	val = readl(spi_regs + SPI_CS);
	val |= SPI_CS_TA;
	writel(val, spi_regs + SPI_CS);/* CS_TA = 1 */
	writel(data & 0xff, spi_regs + SPI_FIFO);	/* TA为1时写入FIFO即为发送 */
	while(!readl(spi_regs + SPI_CS) | SPI_CS_DONE);/* 等待发送完成 */
	val = readl(spi_regs + SPI_CS);
	val &= ~SPI_CS_TA;
	writel(val, spi_regs + SPI_CS);/* CS_TA = 0 */
}

// 写命令函数
static void LCD_WR_REG(u8 cmd)
{
    gpio_direction_output(DC_IO, 0); // DC为低时写命令
	gpio_direction_output(CS_IO, 0);
    spi_write_byte(cmd);
	gpio_direction_output(CS_IO, 1);
}

// 写数据函数
static void LCD_WR_DATA(u8 data)
{
    gpio_direction_output(CS_IO, 0);
    gpio_direction_output(DC_IO, 1); // DC为高时写数据
    spi_write_byte(data);
	gpio_direction_output(CS_IO, 1);
}

// 向LCD寄存器内写入一个数据
static void LCD_WriteReg(u8 LCD_Reg, u16 LCD_RegValue)
{	
	LCD_WR_REG(LCD_Reg);  
	LCD_WR_DATA(LCD_RegValue);	    		 
}	   

// 写入一个16位的数据
static void Lcd_WriteData_16Bit(u16 data)
{	
   	gpio_direction_output(CS_IO, 0);
   	gpio_direction_output(DC_IO, 1);
   	spi_write_byte(data>>8);
	spi_write_byte(data);
   	gpio_direction_output(CS_IO, 1);
}

// 准备写入GRAM
void LCD_WriteRAM_Prepare(void)
{
	LCD_WR_REG(0x2c);
}	 

// 数据写入屏幕的位置
static void LCD_SetWindows(u16 xStar, u16 yStar,u16 xEnd,u16 yEnd)
{	
	LCD_WR_REG(0x2a);	
	LCD_WR_DATA(xStar>>8);
	LCD_WR_DATA(0x00FF&xStar);		
	LCD_WR_DATA(xEnd>>8);
	LCD_WR_DATA(0x00FF&xEnd);

	LCD_WR_REG(0x2b);	
	LCD_WR_DATA(yStar>>8);
	LCD_WR_DATA(0x00FF&yStar);		
	LCD_WR_DATA(yEnd>>8);
	LCD_WR_DATA(0x00FF&yEnd);

	LCD_WriteRAM_Prepare();	//开始写入GRAM			
}   

// 初始化TFT屏幕
static void tft_init(void)
{
	// 复位屏幕
    gpio_direction_output(RST_IO, 0);
    mdelay(100);
    gpio_set_value(RST_IO, 1);
    mdelay(50);

//*************2.8inch ILI9341初始化**********//	
	LCD_WR_REG(0xCF);  
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0xC9); //C1 
	LCD_WR_DATA(0x30); 
	LCD_WR_REG(0xED);  
	LCD_WR_DATA(0x64); 
	LCD_WR_DATA(0x03); 
	LCD_WR_DATA(0x12); 
	LCD_WR_DATA(0x81); 
	LCD_WR_REG(0xE8);  
	LCD_WR_DATA(0x85); 
	LCD_WR_DATA(0x10); 
	LCD_WR_DATA(0x7A); 
	LCD_WR_REG(0xCB);  
	LCD_WR_DATA(0x39); 
	LCD_WR_DATA(0x2C); 
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x34); 
	LCD_WR_DATA(0x02); 
	LCD_WR_REG(0xF7);  
	LCD_WR_DATA(0x20); 
	LCD_WR_REG(0xEA);  
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x00); 
	LCD_WR_REG(0xC0);    //Power control 
	LCD_WR_DATA(0x1B);   //VRH[5:0] 
	LCD_WR_REG(0xC1);    //Power control 
	LCD_WR_DATA(0x00);   //SAP[2:0];BT[3:0] 01 
	LCD_WR_REG(0xC5);    //VCM control 
	LCD_WR_DATA(0x30); 	 //3F
	LCD_WR_DATA(0x30); 	 //3C
	LCD_WR_REG(0xC7);    //VCM control2 
	LCD_WR_DATA(0xB7); 
	LCD_WR_REG(0x36);    // Memory Access Control 
	LCD_WR_DATA(0x08); 
	LCD_WR_REG(0x3A);   
	LCD_WR_DATA(0x55); 
	LCD_WR_REG(0xB1);   
	LCD_WR_DATA(0x00);   
	LCD_WR_DATA(0x1A); 
	LCD_WR_REG(0xB6);    // Display Function Control 
	LCD_WR_DATA(0x0A); 
	LCD_WR_DATA(0xA2); 
	LCD_WR_REG(0xF2);    // 3Gamma Function Disable 
	LCD_WR_DATA(0x00); 
	LCD_WR_REG(0x26);    //Gamma curve selected 
	LCD_WR_DATA(0x01); 
	LCD_WR_REG(0xE0);    //Set Gamma 
	LCD_WR_DATA(0x0F); 
	LCD_WR_DATA(0x2A); 
	LCD_WR_DATA(0x28); 
	LCD_WR_DATA(0x08); 
	LCD_WR_DATA(0x0E); 
	LCD_WR_DATA(0x08); 
	LCD_WR_DATA(0x54); 
	LCD_WR_DATA(0xA9); 
	LCD_WR_DATA(0x43); 
	LCD_WR_DATA(0x0A); 
	LCD_WR_DATA(0x0F); 
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x00); 		 
	LCD_WR_REG(0xE1);    //Set Gamma 
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x15); 
	LCD_WR_DATA(0x17); 
	LCD_WR_DATA(0x07); 
	LCD_WR_DATA(0x11); 
	LCD_WR_DATA(0x06); 
	LCD_WR_DATA(0x2B); 
	LCD_WR_DATA(0x56); 
	LCD_WR_DATA(0x3C); 
	LCD_WR_DATA(0x05); 
	LCD_WR_DATA(0x10); 
	LCD_WR_DATA(0x0F); 
	LCD_WR_DATA(0x3F); 
	LCD_WR_DATA(0x3F); 
	LCD_WR_DATA(0x0F); 
	LCD_WR_REG(0x2B); 
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x01);
	LCD_WR_DATA(0x3f);
	LCD_WR_REG(0x2A); 
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0xef);	 
	LCD_WR_REG(0x11); //Exit Sleep
	mdelay(120);
	LCD_WR_REG(0x29); //display on	
	LCD_WriteReg(0x36,(1<<3)|(0<<7)|(1<<6)|(1<<5));//BGR==1,MY==1,MX==0,MV==1，90度偏转，横屏	
	gpio_direction_output(LED_IO, 0);	// 点亮屏幕
}

/* 写一帧图像 */
static void tft_lcdfb_imageblit(struct fb_info *info, const struct fb_image *image)
{
	int x, y;
    u32 k;
    u32 *p = (u32 *)(info->screen_base);
    u16 c;
    u8 *pp;

    LCD_SetWindows(image->dx, image->dy, image->dx + image->width - 1, image->dy + image->height - 1);
    for (y = 0; y < info->var.yres; y++) {
        for (x = 0; x < info->var.xres; x++) {
            k = p[y*info->var.xres+x];//取出一个像素点的32位数据
            // rgb8888 --> rgb565       
            pp = (u8 *)&k;  
            c = pp[0] >> 3; //蓝色
            c |= (pp[1]>>2)<<5; //绿色
            c |= (pp[2]>>3)<<11; //红色

            //发出像素数据的rgb565
            Lcd_WriteData_16Bit(c);
        }
    }
}

/*调色板函数*/
static int tft_lcdfb_setcolreg(unsigned int regno,unsigned int red,unsigned int green,unsigned int blue,unsigned int transp,struct fb_info *info)
{
	return 0;
}

static int lcd_init(void)
{
	int retval;
	int pin;
	u32 *gpio;

	/*1.分配一个fb_info 
	 *其中分配时fb_info里面的值默认都是0，所有下面有些参数可以不用设置默认0
	 */
	tft_lcd = framebuffer_alloc(0,NULL);		/*其中0代表不需要额外的私有数据空间*/
	/*2.设置*/
	/*2.1设置固定的参数*/
	strcpy(tft_lcd->fix.id,"mylcd");		/*设置fix的名称*/
	tft_lcd->fix.smem_len		= 240*320*16;		/*按具体的屏幕--设置一帧的大小*/
	tft_lcd->fix.type	 		= FB_TYPE_PACKED_PIXELS;/*默认值0*/
	tft_lcd->fix.visual	 		= FB_VISUAL_TRUECOLOR;	/*TFT真彩色*/
	tft_lcd->fix.line_length 	= 240*2;		/*一行的长度大小。。单位byte*/
	/*2.2设置可变的参数*/
	tft_lcd->var.xres			= 240;		/*x方向的分辨率*/
	tft_lcd->var.yres			= 320;		/*y方向的分辨率*/
	tft_lcd->var.xres_virtual	= 240;		/*x方向的虚拟分辨率*/
	tft_lcd->var.xres_virtual	= 320;		/*y方向的虚拟分辨率*/
	tft_lcd->var.bits_per_pixel	= 16;		/*每个像素16位--bpp*/
 
	/*RGB--5:6:5*/
	tft_lcd->var.red.offset		= 11;		/*第11位开始*/
	tft_lcd->var.red.length		= 5;		/*长度5位*/
	
	tft_lcd->var.green.offset	= 5;
	tft_lcd->var.green.length	= 6;
 
	tft_lcd->var.blue.offset	= 0;
	tft_lcd->var.blue.length	= 5;
 
	tft_lcd->var.activate		= FB_ACTIVATE_NOW;
 
	/*2.3设置操作函数*/
	tft_lcd->fbops				= &tft_lcdfb_ops;	
 
	/*2.4其他的设置*/
 
	//s3c_lcd->screen_base		=;		/*显存的虚拟地址*/
	tft_lcd->screen_size		= 240*320*2;	/*屏幕的大小，单位byte*/
 
	/*3.硬件相关的设置*/
	/*3.1配置GPIO用于LCD*/
	retval = gpio_request(DC_IO, "tft_dc");
	if (retval < 0) {
		printk("fb_tft: GPIO%d request failed\n", DC_IO);
		return retval;
	}
	retval = gpio_request(CS_IO, "tft_cs");
	if (retval < 0) {
		printk("fb_tft: GPIO%d request failed\n", CS_IO);
		return retval;
	}
	retval = gpio_request(RST_IO, "tft_rst");
	if (retval < 0) {
		printk("fb_tft: GPIO%d request failed\n", RST_IO);
		return retval;
	}
	retval = gpio_request(LED_IO, "tft_rst");
	if (retval < 0) {
		printk("fb_tft: GPIO%d request failed\n", LED_IO);
		return retval;
	}
	gpio_direction_output(DC_IO, 1);
	gpio_direction_output(RST_IO, 1);
	gpio_direction_output(CS_IO, 1);
	gpio_direction_output(LED_IO, 1);
 
	/*3.2根据手册设置SPI控制器，比如CLK的频率等*/
	/* SPI的GPIO初始化 */
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))
	gpio = ioremap(GPIO_BASE, SZ_16K);
	/* SPI is on GPIO 9..11 */
	for (pin = 9; pin <= 11; pin++) {
		INP_GPIO(pin);		/* set mode to GPIO input first */
		SET_GPIO_ALT(pin, 0);	/* set mode to ALT 0 */
	}
	iounmap(gpio);
#undef INP_GPIO
#undef SET_GPIO_ALT
	spi_regs = ioremap(SPI_BASE, sizeof(spi_regs_t));
	writel(0x0, spi_regs + SPI_CS);	/* MODE0，不使能DMA，一次写一个字节 */
	writel(16, spi_regs + SPI_CLK);	/* 16分频给SPI时钟 */
	/*申请一块连续的内存，返回虚拟地址*/
	tft_lcd->screen_base = dma_alloc_coherent(NULL,tft_lcd->fix.smem_len,(dma_addr_t *)&tft_lcd->fix.smem_start,GFP_KERNEL);
 
	//s3c_lcd->fix.smem_start=xxx;/*显存的物理地址*/

	/*4.注册*/
	register_framebuffer(tft_lcd);

	tft_init();	// 初始化屏幕

	return 0;
}

static void lcd_exit(void)
{
	unregister_framebuffer(tft_lcd);
	gpio_set_value(LED_IO, 0);	/*关闭背光*/
	dma_free_coherent(NULL,tft_lcd->fix.smem_len,(dma_addr_t *)tft_lcd->screen_base,tft_lcd->fix.smem_start);
	iounmap(spi_regs);
	gpio_free(DC_IO);
	gpio_free(CS_IO);
	gpio_free(LED_IO);
	gpio_free(LED_IO);
	framebuffer_release(tft_lcd);
}

module_init(lcd_init);
module_exit(lcd_exit);
 
MODULE_LICENSE("GPL");
