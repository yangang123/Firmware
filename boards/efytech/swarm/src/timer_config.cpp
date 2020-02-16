
#include <px4_arch/io_timer_hw_description.h>

constexpr io_timers_t io_timers[MAX_IO_TIMERS] = {
	initIOTimer(Timer::Timer3),
	initIOTimer(Timer::Timer4),
};

constexpr timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
	initIOTimerChannel(io_timers, {Timer::Timer3, Timer::Channel1}, {GPIO::PortA, GPIO::Pin6}),
	initIOTimerChannel(io_timers, {Timer::Timer3, Timer::Channel2}, {GPIO::PortA, GPIO::Pin7}),
	initIOTimerChannel(io_timers, {Timer::Timer3, Timer::Channel3}, {GPIO::PortB, GPIO::Pin0}),
	initIOTimerChannel(io_timers, {Timer::Timer3, Timer::Channel4}, {GPIO::PortB, GPIO::Pin1}),
	initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel1}, {GPIO::PortD, GPIO::Pin12}),
	initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel2}, {GPIO::PortD, GPIO::Pin13}),
	initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel3}, {GPIO::PortD, GPIO::Pin14}),
	initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel4}, {GPIO::PortD, GPIO::Pin15}),

};

constexpr io_timers_channel_mapping_t io_timers_channel_mapping =
	initIOTimerChannelMapping(io_timers, timer_io_channels);


#if defined(BOARD_HAS_LED_PWM) || defined(BOARD_HAS_UI_LED_PWM)
constexpr io_timers_t led_pwm_timers[MAX_LED_TIMERS] = {
#  if defined(BOARD_HAS_UI_LED_PWM)
	initIOTimer(Timer::Timer5),
#  endif
#  if defined(BOARD_HAS_LED_PWM) && !defined(BOARD_HAS_CONTROL_STATUS_LEDS)
	initIOTimer(Timer::Timer3),
#  endif
};

/* Support driving active low (preferred) or active high LED
 * on both the onboard status LEDs or the [n]UI_LED_<color>[_EXTERNAL]
 *
 * Use open drain to drive the LED. This will ensure that
 * if the LED has a 5 Volt supply that the LED will be
 * off when high.
 */
#define CCER_C1_NUM_BITS   4
#define ACTIVE_LOW(c)      (GTIM_CCER_CC1P << (((c)-1) * CCER_C1_NUM_BITS))
#define ACTIVE_HIGH(c)     0

#if defined(BOARD_LED_PWM_DRIVE_ACTIVE_LOW)
#  define POLARITY(c)      ACTIVE_LOW(c)
#  define DRIVE_TYPE(p)    ((p)|GPIO_OPENDRAIN)
#else
#  define POLARITY(c)      ACTIVE_HIGH((c))
#  define DRIVE_TYPE(p)    (p)
#endif

#if defined(BOARD_UI_LED_PWM_DRIVE_ACTIVE_LOW)
#  define UI_POLARITY(c)    ACTIVE_LOW(c)
#  define UI_DRIVE_TYPE(p)  ((p)|GPIO_OPENDRAIN)
#else
#  define UI_POLARITY(c)    ACTIVE_HIGH((c))
#  define UI_DRIVE_TYPE(p)  (p)
#endif

static inline constexpr timer_io_channels_t initIOTimerChannelUILED(const io_timers_t io_timers_conf[MAX_LED_TIMERS],
		Timer::TimerChannel timer, GPIO::GPIOPin pin, int ui_polarity)
{
	timer_io_channels_t ret = initIOTimerChannel(io_timers_conf, timer, pin);
	ret.gpio_out = UI_DRIVE_TYPE(ret.gpio_out);
	ret.masks = UI_POLARITY(ui_polarity);
	return ret;
}

static inline constexpr timer_io_channels_t initIOTimerChannelControlLED(const io_timers_t
		io_timers_conf[MAX_LED_TIMERS],
		Timer::TimerChannel timer, GPIO::GPIOPin pin, int polarity)
{
	timer_io_channels_t ret = initIOTimerChannel(io_timers_conf, timer, pin);
	ret.gpio_out = DRIVE_TYPE(ret.gpio_out);
	ret.masks = POLARITY(polarity);
	return ret;
}

constexpr timer_io_channels_t led_pwm_channels[MAX_TIMER_LED_CHANNELS] = {
#  if defined(BOARD_HAS_UI_LED_PWM)
#    if defined(BOARD_UI_LED_SWAP_RG)
	initIOTimerChannelUILED(led_pwm_timers, {Timer::Timer5, Timer::Channel2}, {GPIO::PortH, GPIO::Pin11}, 2),
	initIOTimerChannelUILED(led_pwm_timers, {Timer::Timer5, Timer::Channel1}, {GPIO::PortH, GPIO::Pin10}, 1),
	initIOTimerChannelUILED(led_pwm_timers, {Timer::Timer5, Timer::Channel3}, {GPIO::PortH, GPIO::Pin12}, 3),
#    else
	initIOTimerChannelUILED(led_pwm_timers, {Timer::Timer5, Timer::Channel1}, {GPIO::PortH, GPIO::Pin10}, 1),
	initIOTimerChannelUILED(led_pwm_timers, {Timer::Timer5, Timer::Channel2}, {GPIO::PortH, GPIO::Pin11}, 2),
	initIOTimerChannelUILED(led_pwm_timers, {Timer::Timer5, Timer::Channel3}, {GPIO::PortH, GPIO::Pin12}, 3),
#    endif
#  endif
#  if defined(BOARD_HAS_LED_PWM) && !defined(BOARD_HAS_CONTROL_STATUS_LEDS)
	initIOTimerChannelControlLED(led_pwm_timers, {Timer::Timer3, Timer::Channel4}, {GPIO::PortB, GPIO::Pin1}, 4),
	initIOTimerChannelControlLED(led_pwm_timers, {Timer::Timer3, Timer::Channel1}, {GPIO::PortC, GPIO::Pin6}, 1),
	initIOTimerChannelControlLED(led_pwm_timers, {Timer::Timer3, Timer::Channel2}, {GPIO::PortC, GPIO::Pin7}, 2),
#  endif
};
#endif // BOARD_HAS_LED_PWM || BOARD_HAS_UI_LED_PWM
