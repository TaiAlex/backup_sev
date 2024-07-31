// axios.js

import axios from "axios";
// import useNavigate from "react-router-dom";

const instance = axios.create({
  baseURL: "http://34.87.151.244:1506", // Đổi thành URL của API của bạn
});

// Add a response interceptor
instance.interceptors.response.use(
  (response) => {
    return response;
  },
  (error) => {
    if (error.response && error.response.status === 401) {
      localStorage.clear();
      console.log("token:", localStorage.getItem("token"));
      // Redirect to http://localhost:7000/ for 401 responses
      window.location.href = "http://34.87.151.244:5173/";
      // const navigate = useNavigate('/');
    }
    return Promise.reject(error);
  }
);

export default instance;
