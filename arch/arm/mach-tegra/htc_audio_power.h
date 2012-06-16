struct aic3008_power {
	spinlock_t spin_lock;
	bool isPowerOn;
	bool mic_switch;
	bool amp_switch;
	void (*powerinit)(void);
	void (*resume)(void);
	void (*suspend)(void);
	void (*mic_powerup)(void);
	void (*mic_powerdown)(void);
	void (*amp_powerup)(int);
	void (*amp_powerdown)(int);
};

void power_config(const char *name, int pin, int method);
void power_deconfig(const char *name, int pin, int method);
void sfio_config(const char *name, int pin);
void sfio_deconfig(const char *name, int pin);
void common_init(void);
void common_config(void);
void common_deconfig(void);

enum GPIO_METHOD {
	REGULATOR_METHOD = 0,
	GPIO_OUTPUT,
	GPIO_INPUT,
	INIT_OUTPUT_LOW,
	INIT_OUTPUT_HIGH,
	INIT_INPUT,
};

enum AMPLIFIER_TYPE {
	HEADSET_AMP = 0,
	SPEAKER_AMP,
	DOCK_AMP,
};
