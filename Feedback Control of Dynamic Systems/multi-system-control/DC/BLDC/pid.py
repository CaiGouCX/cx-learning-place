class PID:
    def __init__(self, kp, ki, kd, dt=0.1, output_limits=(0,100)):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt                 = dt
        self.integral           = 0
        self.prev_error         = 0
        self.min_out, self.max_out = output_limits

        
        # 控制器状态
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_output = 0.0
        
        # 抗积分饱和标志
        self.saturation_flag = False

    def reset(self):
        self.integral = 0
        self.prev_error = 0


    def set_max(self,maxv):
        self.min_out = -maxv
        self.max_out = maxv

    def compute(self, setpoint, measured_value):
        """
        计算PID输出
        
        参数:
        setpoint - 设定值
        measured_value - 测量值
        
        返回:
        output - 控制器输出
        """
        # 计算误差
        error = setpoint - measured_value
        
        # 比例项
        proportional = self.kp * error
        
        # 积分项 (带抗饱和处理)
        if not self.saturation_flag:
            # 只有未饱和时才累加积分
            self.integral += error * self.dt
        
        integral_term = self.ki * self.integral
        
        # 微分项 (带滤波)
        derivative = (error - self.prev_error) / self.dt
        derivative_term = self.kd * derivative
        
        # 计算原始输出
        output = proportional + integral_term + derivative_term
        
        # 保存未限幅前的输出用于抗饱和判断
        raw_output = output
        
        # 输出限幅
        if self.min_out is not None:
            output = max(self.min_out, output)
        if self.max_out is not None:
            output = min(self.max_out, output)
        
        # 抗积分饱和逻辑
        self.saturation_flag = False
        if (self.min_out is not None and raw_output < self.min_out) or \
            (self.max_out is not None and raw_output > self.max_out):
            
            # 检测到输出饱和
            self.saturation_flag = True
            
            # 反向修正积分项 (只修正会加剧饱和的部分)
            if (output <= self.min_out and error > 0) or \
                (output >= self.max_out and error < 0):
                # 只在不加剧饱和的方向上允许积分
                self.integral = (output - proportional - derivative_term) / self.ki if self.ki != 0 else 0
        
        # 更新状态
        self.prev_error = error
        self.prev_output = output
        '''print(self.min_out,self.max_out)
        if output == self.min_out or output == self.max_out:
            print("⚠️ PID 输出触顶，可能不够推转！")'''

        return output


