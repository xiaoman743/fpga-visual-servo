@echo off
chcp 65001 >nul
cd /d "%~dp0"
echo.
echo ============================================
echo   推送代码包到 GitHub
echo ============================================
echo.
echo 先做一件事：
echo   1. 浏览器打开 https://github.com/new
echo   2. 仓库名建议：fpga-visual-servo   （不要带学校信息）
echo   3. 选 Public 或 Private 都行
echo   4. 下面的 "Add a README file" / .gitignore / license 一个都不要勾
echo   5. 点 Create repository，然后复制页面上那个以 .git 结尾的地址
echo.
set /p URL=把仓库地址粘贴到这里再回车（例如 https://github.com/xxx/fpga-visual-servo.git）: 
if "%URL%"=="" (
  echo.
  echo 没有输入地址，已退出。
  pause
  exit /b 1
)
echo.
echo [1/3] 设置分支为 main ...
git branch -M main
echo [2/3] 设置远程仓库 ...
git remote remove origin 2>nul
git remote add origin "%URL%"
echo [3/3] 开始推送（可能弹出 GitHub 登录窗口，按提示授权）...
echo.
git push -u origin main
echo.
if errorlevel 1 (
  echo ============================================
  echo  推送失败。常见原因：
  echo   1) 仓库不是空的（勾了 README）——删掉仓库重建，什么都别勾
  echo   2) 没登录 / 授权被拒——再运行一次，弹窗里点 Authorize
  echo   3) 网络问题——确认 VPN 开着
  echo  把上面的报错整段复制给 AI，可以继续排查。
  echo ============================================
) else (
  echo ============================================
  echo  推送成功！
  echo  把上面那个仓库地址填进提交页的 "Github链接输入" 即可。
  echo ============================================
)
pause
