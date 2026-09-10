#include "psyqo/advancedpad.hh"

#include "control/e32_controller.h"


static psyqo::AdvancedPad pad;

static Controller controller[CONTROLLER_COUNT];

Controller *controller_get(void) { return controller; }


/* The first port of each side; multitaps are not read. */
static const psyqo::AdvancedPad::Pad port[CONTROLLER_COUNT] = {
	psyqo::AdvancedPad::Pad::Pad1a,
	psyqo::AdvancedPad::Pad::Pad2a,
};

/* Stick values arrive 0..255 with the centre at 128. */
static int8_t axis(uint8_t value)
{
	return (int8_t)((int)value - 128);
}

static int8_t axisPressed(int8_t now, int8_t before)
{
	int8_t now_dir    = now    >  STICK_DEADZONE ? 1 : now    < -STICK_DEADZONE ? -1 : 0;
	int8_t before_dir = before >  STICK_DEADZONE ? 1 : before < -STICK_DEADZONE ? -1 : 0;
	return now_dir != before_dir ? now_dir : 0;
}

static void controller_getInputs(Controller *c, psyqo::AdvancedPad::Pad p)
{
	c->connected = pad.isPadConnected(p);

	Buttons held = 0;
	for (unsigned b = 0; b < 16; b++)
		if (pad.isButtonPressed(p, (psyqo::AdvancedPad::Button)b)) held |= (Buttons)(1u << b);

	c->pressed  = held & (Buttons)~c->held;
	c->released = c->held & (Buttons)~held;
	c->held     = held;

	uint8_t type = pad.getPadType(p);
	c->analog = type == psyqo::AdvancedPad::PadType::AnalogPad
	         || type == psyqo::AdvancedPad::PadType::AnalogStick;

	/* The pad reports the right stick first, then the left. */
	ControllerInputs input = {};
	if (c->analog) {
		input.right_x = axis(pad.getAdc(p, 0));
		input.right_y = axis(pad.getAdc(p, 1));
		input.left_x  = axis(pad.getAdc(p, 2));
		input.left_y  = axis(pad.getAdc(p, 3));
	}

	c->stick_pressed_x = axisPressed(input.left_x, c->input.left_x);
	c->stick_pressed_y = axisPressed(input.left_y, c->input.left_y);

	c->input = input;
}

static Fixed button(Buttons buttons, psyqo::AdvancedPad::Button b)
{
	return (buttons >> b) & 1 ? 1.0_fp : Fixed();
}

Fixed button_getPressed(const Controller *c, Buttons buttons, ButtonID id)
{
	(void)c;
	using B = psyqo::AdvancedPad::Button;

	switch (id) {
		case BTN_CROSS:    return button(buttons, B::Cross);
		case BTN_CIRCLE:   return button(buttons, B::Circle);
		case BTN_SQUARE:   return button(buttons, B::Square);
		case BTN_TRIANGLE: return button(buttons, B::Triangle);
		case BTN_START:    return button(buttons, B::Start);
		case BTN_SELECT:   return button(buttons, B::Select);
		case BTN_D_UP:     return button(buttons, B::Up);
		case BTN_D_DOWN:   return button(buttons, B::Down);
		case BTN_D_LEFT:   return button(buttons, B::Left);
		case BTN_D_RIGHT:  return button(buttons, B::Right);
		case BTN_L1:       return button(buttons, B::L1);
		case BTN_R1:       return button(buttons, B::R1);
		case BTN_L2:       return button(buttons, B::L2);
		case BTN_R2:       return button(buttons, B::R2);
		case BTN_L3:       return button(buttons, B::L3);
		case BTN_R3:       return button(buttons, B::R3);
		default:           return Fixed();
	}
}

/* -128..127 to -1..1 past the dead zone. */
static Fixed stick(int8_t value)
{
	int v = value;
	int sign = 1;
	if (v < 0) { v = -v; sign = -1; }

	v -= STICK_DEADZONE;
	if (v <= 0) return Fixed();

	int range = 127 - STICK_DEADZONE;
	if (v > range) v = range;

	return Fixed(sign * v * 4096 / range, Fixed::RAW);
}

Fixed axis_get(const Controller *c, AxisID id)
{
	switch (id) {
		case AXIS_LEFT_X:  return stick(c->input.left_x);
		case AXIS_LEFT_Y:  return stick(c->input.left_y);
		case AXIS_RIGHT_X: return stick(c->input.right_x);
		case AXIS_RIGHT_Y: return stick(c->input.right_y);
		default:           return Fixed();
	}
}

void controller_start(void)
{
	for (int i = 0; i < CONTROLLER_COUNT; i++)
		controller[i] = Controller();

	pad.initialize(psyqo::AdvancedPad::PollingMode::Fast);
}

void controller_poll(void)
{
	for (int i = 0; i < CONTROLLER_COUNT; i++)
		controller_getInputs(&controller[i], port[i]);
}
