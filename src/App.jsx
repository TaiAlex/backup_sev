import ReactDOM from "react-dom/client";
import { BrowserRouter, Routes, Route, Outlet } from "react-router-dom";
import Login from "./pages/Login";
// import App from "./App.tsx";
import PrivateRoute from "./PrivateRoute";
import Decentralization from "./pages/Decentralization";
import Home from "./pages/Home";
import Layout from "./Layout";
import Control from "./pages/Control";
import DataAnalysis from "./pages/DataAnalysis";
import Test from "./draft";

const App = () => {
  return (
    <BrowserRouter>
      <Routes>
        <Route index element={<Login />} />
        <Route path="/" element={<Layout />}>
          <Route
            path="/home"
            element={
              <PrivateRoute>
                <Home />
              </PrivateRoute>
            }
          />
          <Route
            path="/control"
            element={
              <PrivateRoute>
                <Control />
              </PrivateRoute>
            }
          />
          <Route
            path="/user-management"
            element={
              <PrivateRoute>
                <Decentralization />
              </PrivateRoute>
            }
          />
          <Route
            path="/Data-Analysis"
            element={
              <PrivateRoute>
                <DataAnalysis />
              </PrivateRoute>
            }
          />
        </Route>
        <Route path="/test" element={<Test />} />
      </Routes>
    </BrowserRouter>
  );
};
export default App;
