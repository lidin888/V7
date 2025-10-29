#!/usr/bin/env python3
import subprocess
import time

def test_binary_sensor():
    """测试传感器二进制文件"""
    print("测试传感器二进制文件...")
    
    try:
        # 运行传感器二进制文件并捕获输出
        process = subprocess.Popen(
            ['./system/sensord/bin/sensord_jy62'],
            cwd='/home/lidin/carrotpilot',
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1
        )
        
        print("传感器进程已启动，等待输出...")
        
        # 等待几秒钟并读取输出
        start_time = time.time()
        lines_read = 0
        
        while time.time() - start_time < 5.0 and lines_read < 10:
            try:
                line = process.stdout.readline()
                if line:
                    print(f"输出: {line.strip()}")
                    lines_read += 1
                else:
                    time.sleep(0.1)
            except Exception as e:
                print(f"读取输出时出错: {e}")
                break
        
        # 终止进程
        process.terminate()
        try:
            process.wait(timeout=1)
        except subprocess.TimeoutExpired:
            process.kill()
            
        print("测试完成")
        
    except Exception as e:
        print(f"测试过程中出错: {e}")

if __name__ == "__main__":
    test_binary_sensor()