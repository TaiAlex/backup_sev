import { useState } from "react";
import axios from "axios";
import { useNavigate } from "react-router-dom";
import { ToastContainer, toast } from "react-toastify";
import "react-toastify/dist/ReactToastify.css";

function App() {
  const [errorValue, setError] = useState(null);
  const navigate = useNavigate();

  const setToken = (token) => {
    localStorage.setItem("token", token);
  };

  const notify = (message) => {
    toast.error(message, {
      position: "top-center", // Position at the top
      autoClose: 3000, // Auto close after 3 seconds
    });
  };

  const postusr = async () => {
    try {
      const response = await axios.post(
        "http://34.87.151.244:1506/token",
        new URLSearchParams({
          username: localStorage.getItem("username"),
          password: localStorage.getItem("password"),
        }),
        {
          headers: {
            accept: "application/json",
            "Content-Type": "application/x-www-form-urlencoded",
          },
        }
      );

      const token = response.data.access_token;
      if (token) {
        setToken(token);
        navigate("/home");
      } else {
        throw new Error("Token not received.");
      }
    } catch (error) {
      console.error("Login error:", error);
      setError("Failed to log in. Please try again.");
      notify("Failed to log in. Please try again.");
    }
  };

  return (
    <div className="flex h-screen w-screen gradient-background-login items-center justify-center">
      <div className="flex flex-col white-box top-1/4 md:top-1/2">
        <div className="container">
          <h1>WELCOME</h1>
        </div>
        <div className="login_box">
          <div className="inline-flex items-center w-full text-sm font-semibold text-black transition-colors duration-150 cursor-pointer hover:text-green-500">
            <svg
              xmlns="http://www.w3.org/2000/svg"
              width="24"
              height="24"
              viewBox="0 0 24 24"
              fill="none"
              stroke="currentColor"
              strokeWidth="2"
              strokeLinecap="round"
              strokeLinejoin="round"
              className="icon icon-tabler icons-tabler-outline icon-tabler-user"
            >
              <path stroke="none" d="M0 0h24v24H0z" fill="none" />
              <path d="M8 7a4 4 0 1 0 8 0a4 4 0 0 0 -8 0" />
              <path d="M6 21v-2a4 4 0 0 1 4 -4h4a4 4 0 0 1 4 4v2" />
            </svg>
            <h2>Username</h2>
          </div>

          <input
            type="text"
            name="username"
            className="login_input mt-1 px-3 py-2 bg-white border shadow-sm border-slate-300 placeholder-slate-300 focus:outline-none focus:border-sky-500 focus:ring-sky-500 block w-full rounded-md sm:text-sm focus:ring-1"
            placeholder="Enter account to log in"
            onChange={(e) => localStorage.setItem("username", e.target.value)}
          />

          <div className="inline-flex items-center w-full text-sm font-semibold text-black transition-colors duration-150 cursor-pointer hover:text-green-500">
            <svg
              xmlns="http://www.w3.org/2000/svg"
              width="24"
              height="24"
              viewBox="0 0 24 24"
              fill="none"
              stroke="currentColor"
              strokeWidth="2"
              strokeLinecap="round"
              strokeLinejoin="round"
            >
              <path stroke="none" d="M0 0h24v24H0z" fill="none" />
              <path d="M5 13a2 2 0 0 1 2 -2h10a2 2 0 0 1 2 2v6a2 2 0 0 1 -2 2h-10a2 2 0 0 1 -2 -2v-6z" />
              <path d="M11 16a1 1 0 1 0 2 0a1 1 0 0 0 -2 0" />
              <path d="M8 11v-4a4 4 0 1 1 8 0v4" />
            </svg>
            <h2>Password</h2>
          </div>
          <input
            type="password"
            name="password"
            className="login_input mt-1 px-3 py-2 bg-white border shadow-sm border-slate-300 placeholder-slate-300 focus:outline-none focus:border-sky-500 focus:ring-sky-500 block w-full rounded-md sm:text-sm focus:ring-1"
            placeholder="Password"
            onChange={(e) => localStorage.setItem("password", e.target.value)}
          />

          <button
            onClick={postusr}
            className="flex-auto button_login_submit bg-green-500 transition-colors duration-150 cursor-pointer hover:bg-green-900 text-cyan-50 font-bold"
          >
            LOGIN
          </button>
        </div>
      </div>
      {errorValue && <ToastContainer />}
    </div>
  );
}

export default App;
