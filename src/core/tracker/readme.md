## 符号约定
https://zeal-up.github.io/2023/03/13/kalman_filter/%E6%89%A9%E5%B1%95%E5%8D%A1%E5%B0%94%E6%9B%BC%E6%BB%A4%E6%B3%A2-%E5%90%AB%E4%BE%8B%E5%AD%90%E5%8F%8A%E4%BB%A3%E7%A0%81/

## Car modle

$$X = [x,v_x,y,v_y,z,v_z,\theta,\omega] ^ T$$
$$Z = [x,v_x,y,v_y,z,v_z,\theta,\omega] ^ T$$


$$ X_k = 
8\left\{\left[
\begin{matrix}
1&dt&...&0&0\\
0&1&...&0&0\\
0&0&...&dt&0\\
0&0&...&1&dt\\
\end{matrix}
\right]\right.
X_{k-1} + w_k
$$