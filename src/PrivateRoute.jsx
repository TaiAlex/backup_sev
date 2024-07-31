import React from "react";
import { Navigate } from "react-router-dom";
import axios from "axios";

const PrivateRoute = ({ children }) => {
  const token = localStorage.getItem("token");
  // console.log("PrivateRoute: ", token);
  return token ? children : <Navigate to="/" />;
};

export default PrivateRoute;
