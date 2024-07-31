_______________________________________________________________________

TẠO SSH CHO MÁY 
ssh-keygen // Tạo ssh 

ssh-keygen -t rsa -f C:\Users\trucd\.ssh\id_rsa -C truc -b 2048 // tạo ssh dùng gg 

_______________________________________________________________________

SET UP MÔI TRƯỜNG SỬ DỤNG REACT 

B1: Cài node js version 18

cd ~
curl -sL https://deb.nodesource.com/setup_18.x -o nodesource_setup.sh
nano nodesource_setup.sh
sudo bash nodesource_setup.sh
sudo apt install nodejs
node -v

https://www.digitalocean.com/community/tutorials/how-to-install-node-js-on-ubuntu-22-04

B2: Cài YARN

npm install --global yarn
yarn --version

https://classic.yarnpkg.com/lang/en/docs/install/#windows-stable

B3: Tạo project 

 - yarn create vite // tạo folder
 - <tên Project> -> React -> JavaScript
 -  cd test
    yarn install
    yarn dev 

B4: Cài Tailwind

npm install -D tailwindcss postcss autoprefixer
npx tailwindcss init -p

sửa file tailwind.config.js
	/** @type {import('tailwindcss').Config} */
	export default {
	  content: [
	    "./index.html",
	    "./src/**/*.{js,ts,jsx,tsx}",
	  ],
	  theme: {
	    extend: {},
	  },
	  plugins: [],
	}

sửa file index.css
	@tailwind base;
	@tailwind components;
	@tailwind utilities;

xóa file App.css

B5: Cai Thuw vien ho tro React
npm i -D react-router-dom@latest
npm i react-chartjs-2 chart.js
npm install @headlessui/react
npm install axios
npm install @fortawesome/react-fontawesome @fortawesome/free-solid-svg-icons @fortawesome/fontawesome-svg-core
npm install @material-tailwind/react
npm i react-phone-input-2
npm install react-icons

B6: Cai thu vien ho tro Server
pip install "passlib[bcrypt]"
pip install pyjwt
pip install fastapi uvicorn python-jose[cryptography] passlib[bcrypt]
pip install fastapi-mail
pip install pyexcel pyexcel-xlsx


_______________________________________________________________________

COPY 
cp -r /path/to/source/directory /path/to/destination/directory


CAP QUYEN CHO USER TUY CHINH
sudo chown -R truc:truc /home/truc/test

WEB SEND API 
https://sabuhish.github.io/fastapi-mail/

WEB CAP App Passwork
https://myaccount.google.com/apppasswords?pli=1&rapt=AEjHL4NopfBmwr4XeCVmCMoudJLJum5h2qPHyNM1QGfWSHwXTnfXyaDVj7SSjOvjaOHcHZL8l-nnoNAgpEv3BGm19pZ-hwExAk3S2vSF2zvYN8Xjl6FKjbk

CSV TO XLSX
pip install pyexcel pyexcel-xlsx

_______________________________________________________________________

WEB tao animation 

https://mambaui.com/components/input
https://tabler.io/icons
https://flowbite.com/docs/forms/toggle/
https://flowbite.com/docs/forms/floating-label/
https://mui.com/material-ui/react-toggle-button/

_______________________________________________________________________
