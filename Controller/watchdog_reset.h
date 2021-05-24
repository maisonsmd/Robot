#pragma once

#define IWDG_PR_DIV_4	0x0
#define IWDG_PR_DIV_8	0x1
#define IWDG_PR_DIV_16	0x2
#define IWDG_PR_DIV_32	0x3
#define IWDG_PR_DIV_64	0x4
#define IWDG_PR_DIV_128	0x5
#define IWDG_PR_DIV_256	0x6

enum iwdg_prescaler {
	IWDG_PRE_4 = IWDG_PR_DIV_4,     /**< Divide by 4 */
	IWDG_PRE_8 = IWDG_PR_DIV_8,     /**< Divide by 8 */
	IWDG_PRE_16 = IWDG_PR_DIV_16,   /**< Divide by 16 */
	IWDG_PRE_32 = IWDG_PR_DIV_32,   /**< Divide by 32 */
	IWDG_PRE_64 = IWDG_PR_DIV_64,   /**< Divide by 64 */
	IWDG_PRE_128 = IWDG_PR_DIV_128, /**< Divide by 128 */
	IWDG_PRE_256 = IWDG_PR_DIV_256  /**< Divide by 256 */
};

struct iwdg_reg_map {
	volatile uint32_t KR;  /**< Key register. */
	volatile uint32_t PR;  /**< Prescaler register. */
	volatile uint32_t RLR; /**< Reload register. */
	volatile uint32_t SR;  /**< Status register */
};

#define IWDG ((iwdg_reg_map *)0x40003000)

void iwdg_feed(void) { IWDG->KR = 0xAAAA; }

void iwdg_init(uint32 prescaler, uint16_t reload) {
	IWDG->KR = 0x5555;
	IWDG->PR = prescaler;
	IWDG->RLR = reload;
	IWDG->KR = 0xCCCC;
	IWDG->KR = 0xAAAA;
}
