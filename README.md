# WIT-SPOT-RESEARCH
# WIT Boston Dynamics Spot Research

## 1. Prerequisites
Before cloning, ensure you have:
* **VS Code** installed.
* **Python 3.9 or 3.10** installed. (Do not use 3.11/3.12 yet, as the SDK has compatibility issues).
* **Git** installed.

## 2. Initial Setup (Do this once)

### Step A: Clone the Repository
Open your terminal/command prompt:
```bash
git clone [https://github.com/robertsona1-arch/WIT-SPOT-RESEARCH.git](https://github.com/robertsona1-arch/WIT-SPOT-RESEARCH.git)
cd WIT-SPOT-RESEARCH
```

### Step B: Create Virtual Environment
We do not share virtual environments. You must create your own local "toolbox."

**For Mac:**
```bash
python3 -m venv spot-env
source spot-env/bin/activate
```

**For Windows (PowerShell):**
```powershell
py -3.10 -m venv spot-env
.\spot-env\Scripts\activate
```
*(If you get a permission error on Windows, run: `Set-ExecutionPolicy RemoteSigned -Scope CurrentUser`)*

### Step C: Install Dependencies
With your environment activated (you should see `(spot-env)` in your terminal), run:
```bash
pip install -r requirements.txt
```

### Step D: Setup Credentials
1.  Look for the file `credentials_template.py`.
2.  **Duplicate it** and rename the copy to `credentials.py`.
3.  Open `credentials.py` and enter the robot's IP, Username, and Password.
4.  *Note: `credentials.py` is ignored by Git, so your secrets will never be uploaded.*

## 3. How to Run Code
1.  Open VS Code in this folder (`code .`).
2.  **Select Interpreter:** Press `Ctrl+Shift+P` (Windows) or `Cmd+Shift+P` (Mac) -> "Python: Select Interpreter" -> Select the `spot-env` you just created.
3.  Open a script (e.g., `stand_and_walk.py`) and press **F5**.

## 4. How to Push Changes
When you have made edits and want to save them to GitHub:

1.  **Pull first** (to get anyone else's changes):
    ```bash
    git pull origin main
    ```
2.  **Stage your files:**
    ```bash
    git add .
    ```
3.  **Commit** (Label your work):
    ```bash
    git commit -m "Added autowalk logic"
    ```
4.  **Push:**
    ```bash
    git push origin main
    ```
