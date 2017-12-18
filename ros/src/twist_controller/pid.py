
MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = mn
        self.max = mx

        self.int_val = self.last_int_val = self.last_error = 0.

    def reset(self):
        self.int_val = 0.0
        self.last_int_val = 0.0

    def step(self, error, sample_time):
        self.last_int_val = self.int_val

        integral = self.int_val + error * sample_time;
        derivative = (error - self.last_error) / sample_time;

        y = self.kp * error + self.ki * self.int_val + self.kd * derivative;
        val = max(self.min, min(y, self.max))

        if val > self.max:
            val = self.max
        elif val < self.min:
            val = self.min
        else:
            self.int_val = integral
        self.last_error = error

        return val

    #def twiddle(cte, tol=0.2): 
    #    p = [0, 0, 0]
    #    dp = [1, 1, 1]
    #    err = 0
    #    best_err = cte
        # twiddle loop here
    #    it = 0
    #    while (sum(dp)) > tol:
    #        for i in range(len(p)):
    #            p[i] += dp[i]
    #            if err < best_err:
    #                best_err = err
    #                dp[i] *= 1.1
    #            else:
    #                p[i] -= 2*dp[i]
    #                
    #                if err < best_err:
    #                    best_err = err
    #                    dp[i] *= 1.1
    #                else:
    #                    p[i] += dp[i]
    #                    dp[i] *= 0.9
    #    it += 1
    #    return p, best_err 
