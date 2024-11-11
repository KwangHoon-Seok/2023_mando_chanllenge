class PIDController:
   

    def update(self, kp,ki,kd,error, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0
        # 오차에 대한 비례항 계산
        proportional = self.kp * error
        
        # 오차에 대한 적분항 계산
        self.integral += error * dt
        integral = self.ki * self.integral
        
        # 오차에 대한 미분항 계산
        derivative = self.kd * (error - self.previous_error) / dt
        self.previous_error = error
        
        # PID 출력 계산
        output = proportional + integral + derivative
        
        return output
        