inline float bias_x(float temp) {
	// Generated 25-Jul-2017
	// Coefficients
	static const uint8_t ORDER = 10;
	static const float COEFF[] = {
			-0.035998,
			0.003294,
			0.250354,
			0.019032,
			-0.636098,
			-0.130806,
			0.781083,
			0.129861,
			-0.450353,
			0.047737,
			0.023253 };
	// Scale and translate temp
	float x = (temp - 149.000000) / 17.752934;
	// Calculate bias
	float xpow = x;
	float bias = COEFF[0];
	for (uint8_t i = 1; i <= ORDER; i++) {
		bias += xpow * COEFF[i];
		xpow *= x;
	}
	return bias;
}

inline float bias_y(float temp) {
	// Generated 25-Jul-2017
	// Coefficients
	static const uint8_t ORDER = 10;
	static const float COEFF[] = {
			0.033907,
			0.024170,
			-0.220314,
			-0.174728,
			0.480334,
			0.407903,
			-0.393520,
			-0.351670,
			0.081421,
			0.218095,
			0.032856 };
	// Scale and translate temp
	float x = (temp - 149.000000) / 17.752934;
	// Calculate bias
	float xpow = x;
	float bias = COEFF[0];
	for (uint8_t i = 1; i <= ORDER; i++) {
		bias += xpow * COEFF[i];
		xpow *= x;
	}
	return bias;
}
