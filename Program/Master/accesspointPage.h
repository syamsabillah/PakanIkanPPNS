#ifndef ACCESSPOINTPAGE_H
#define ACCESSPOINTPAGE_H

const char *htmlPage = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>WiFi Login</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            background-color: #f4f4f4;
        }

        .container {
            width: 300px;
            margin: 0 auto;
            padding: 20px;
            background-color: #fff;
            border-radius: 5px;
            box-shadow: 0 0 10px rgba(0, 0, 0, 0.1);
            margin-top: 100px;
        }

        h2 {
            text-align: center;
        }

        .form-group {
            margin-bottom: 20px;
        }

        label {
            display: block;
            font-weight: bold;
        }

        input[type="text"],
        input[type="password"] {
            width: 90%;
            padding: 10px;
            border: 1px solid #ccc;
            border-radius: 5px;
        }

        input[type="submit"] {
            background-color: #007BFF;
            color: #fff;
            padding: 10px 20px;
            border: none;
            border-radius: 5px;
            cursor: pointer;
        }

        input[type="submit"]:hover {
            background-color: #0056b3;
        }
    </style>
</head>
<body>
    <div class="container">
        <h2>WiFi Login</h2>
        <form action="/submit" method="post">
            <div class="form-group">
                <label for="SSID">SSID:</label>
                <input type="text" id="SSID" name="SSID" required>
            </div>
            <div class="form-group">
                <label for="password">Password:</label>
                <input type="password" id="password" name="password">
                <input type="checkbox" id="showPassword"> Show Password<br>
            </div>
            <div class="form-group">
                <input type="submit" value="Save and Restart">
            </div>
        </form>
        <script>
        const passwordInput = document.getElementById('password');
        const showPasswordCheckbox = document.getElementById('showPassword');

        showPasswordCheckbox.addEventListener('change', function () {
            if (showPasswordCheckbox.checked) {
                passwordInput.type = 'text';
            } else {
                passwordInput.type = 'password';
            }
        });
    </script>
    </div>
</body>
</html>
)rawliteral";

#endif // ACCESSPOINTPAGE_H
