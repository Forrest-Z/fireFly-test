## PID粗略的理解
>**P**代表比例--用来放大error，反馈到目标量
</br>**I**代表积分--用来消除系统误差
</br>**D**代表微分--用来消除震荡</br>
+ 数学表达式</br>
<a href="http://www.codecogs.com/eqnedit.php?latex=G_{t}=P\times&space;Error_{t}&plus;D\times&space;\frac{\mathrm{d}&space;Error_{t}}{\mathrm{d}&space;t}&plus;\sum_{0}^{t}Error_{t}" target="_blank"><img src="http://latex.codecogs.com/gif.latex?G_{t}=P\times&space;Error_{t}&plus;D\times&space;\frac{\mathrm{d}&space;Error_{t}}{\mathrm{d}&space;t}&plus;\sum_{0}^{t}Error_{t}" title="G_{t}=P\times Error_{t}+D\times \frac{\mathrm{d} Error_{t}}{\mathrm{d} t}+\sum_{0}^{t}Error_{t}" /></a>
## PID参数的确定
+ TWIDDLE 算法
>### 算法思路
>1. 给定一个参数p，要优化达到最佳情况，每次步进dp(增加或者减少)
>2. 每次更新参数p后，计算一下新的效果，并且与之前最好的效果比较，如果效果理想，则结束
>3. 如果更新完后效果变好，则继续朝着当前方向（dp同时变大）更新;如果效果变差则朝着反方向更新
>4. 如果两个方向都尝试过了,并且效果都比不上之前，则回到上一次效果最好时的p，并且减小dp
>5. 循环以上步骤
---
> ### 伪代码
```
  if(error<best_error)
    best_error=error
    保持上一次的方向（增加或者减小）
    dp*=1.05
    失败次数=0
  else if (失败次数<2)
    上一次增加则执行 p-=dp*2
    上一次减少则执行 p+=dp*2
    失败次数++
  else
    上一次是增加则执行 p-=dp
    上一次是减少则执行 p+=dp
    dp*=0.95
```
> ### PID.CPP（整个文件可以看CarND-PID-Control-Project文件夹)
```c++
void PID::UpdateError(double cte) {
    p_error=cte;
    d_error=cte-last_cte;
    i_error+=cte;
    last_cte=cte;
    times++;

    if(Twiddle)
    {
        error_sum+=abs(cte);
        if(times%twiddle_loop==0 )
        {
            error_sum/=twiddle_loop;
            if(error_sum<best_error)
            {
                best_error=error_sum;
                pidState[twiddle_times].fail_times=0;
                dp[twiddle_times]*=1.05;
                p[twiddle_times]+=pidState[twiddle_times].last_state*dp[twiddle_times];
            }
            else if(pidState[twiddle_times].fail_times<2)
            {
                pidState[twiddle_times].last_state*=-1;
                p[twiddle_times]+=2*pidState[twiddle_times].last_state*dp[twiddle_times];
                pidState[twiddle_times].fail_times++;
            }
            else
            {
                pidState[twiddle_times].fail_times=0;
                pidState[twiddle_times].last_state*=-1;
                p[twiddle_times]+=pidState[twiddle_times].last_state*dp[twiddle_times];
                dp[twiddle_times]*=0.95;
                twiddle_times++;
                twiddle_times%=3;
            }
          cout<<"P:"<<p[0]<<" I:"<<p[1]<<" D:"<<p[2]<<" "<<error_sum<<endl;
          error_sum=0;
        }

    }
}
```
### 总结
1. twiddle的资料比较难找，我不太清楚我的代码是否正确
2. 开启twiddle的效果和直接使用常量参数的效果差不多
3. 参数的设置比较随意，主要是用来了解一下pid的工作流程
4. 开启I(即积分项)会翻车，思考了一下，主要是因为在高速公路这种目标的动态变化的，在某些时刻误差会偏向某一个方向，积分项会迅速朝着一个方向增长，此种情况
下关闭，I，即使用PD控制，效果更好
