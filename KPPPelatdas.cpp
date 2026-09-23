#include <bits/stdc++.h>
#include <vector>
using namespace std;

/*
    Variable Dictionary :
    1. maze = labirin
    2. gScore = jarak (jumlah langkah) terpendek yang ditemukan dari start ke sel itu
    3. parent = sel asal, dipakai untuk menyusun ulang jalur di akhir
    4. h = heuristik (perkiraan jarak sel ke target)
*/

// Struct untuk melacak posisi robot di priority queue
struct nodePos {
    int rowPos;
    int colPos;
    int stepCost; // ini "g" -> jumlah langkah dari start
    int h;        // perkiraan jarak ke target

    int f() const { return stepCost + h; }

    // supaya priority_queue tahu cara membandingkan: f terkecil diproses duluan
    bool operator>(const nodePos& other) const {
        return f() > other.f();
    }
};

// Mencari posisi dari start, flag, dan goal
// ini juga berlaku di luar (di main), bukan cuma copy lokal
void findPos(vector<vector<char>> &maze, pair<int,int> &startPos, pair<int,int> &flagPos, pair<int,int> &goalPos, int ROWS, int COLS) {
    for (int i = 0; i < ROWS; i++) {
        for (int j = 0; j < COLS; j++) {
            char isi_sel = maze[i][j];
            if (isi_sel == 'S') startPos = {i, j};
            else if (isi_sel == 'F') flagPos = {i, j};
            else if (isi_sel == 'G') goalPos = {i, j};
        }
    }
}

// Cek apakah sebuah sel valid untuk dilewati
// # = dinding, X = bom -> dua-duanya tidak boleh dilewati
bool isValidMove(vector<vector<char>>& maze, int xPos, int yPos) {
    int totRow = maze.size();
    int totCol = maze[0].size();

    if (xPos < 0 || xPos >= totRow || yPos < 0 || yPos >= totCol) return false;

    char isi_sel = maze[xPos][yPos];
    if (isi_sel == '#' || isi_sel == 'X') return false;

    return true;
}

// Heuristik: perkiraan jarak dari (r1,c1) ke (r2,c2)
int hitungHeuristik(int r1, int c1, int r2, int c2) {
    return abs(r1 - r2) + abs(c1 - c2);
}

// Fungsi utama A*: cari jalur terpendek dari start ke target
// Mengembalikan urutan koordinat (kosong kalau tidak ada jalan)
vector<pair<int,int>> jalankanAstar(vector<vector<char>>& maze, pair<int,int> start, pair<int,int> target) {
    int ROWS = maze.size();
    int COLS = maze[0].size();

    // arah gerak: atas, bawah, kiri, kanan
    int dRow[] = {-1, 1, 0, 0};
    int dCol[] = {0, 0, -1, 1};

    vector<vector<int>> gScore(ROWS, vector<int>(COLS, INT_MAX));
    vector<vector<pair<int,int>>> parent(ROWS, vector<pair<int,int>>(COLS, {-1,-1}));
    vector<vector<bool>> visited(ROWS, vector<bool>(COLS, false));

    priority_queue<nodePos, vector<nodePos>, greater<nodePos>> antrian;

    gScore[start.first][start.second] = 0;
    antrian.push({start.first, start.second, 0, hitungHeuristik(start.first, start.second, target.first, target.second)});

    while (!antrian.empty()) {
        nodePos sekarang = antrian.top();
        antrian.pop();

        // kalau sel ini sudah pernah diproses, skip (sudah ada jalur lebih baik/setara)
        if (visited[sekarang.rowPos][sekarang.colPos]) continue;
        visited[sekarang.rowPos][sekarang.colPos] = true;

        // sudah sampai target? berhenti
        if (sekarang.rowPos == target.first && sekarang.colPos == target.second) break;

        // coba 4 arah
        for (int i = 0; i < 4; i++) {
            int newRow = sekarang.rowPos + dRow[i];
            int newCol = sekarang.colPos + dCol[i];

            if (!isValidMove(maze, newRow, newCol) || visited[newRow][newCol]) continue;

            int gBaru = gScore[sekarang.rowPos][sekarang.colPos] + 1;

            // kalau jalur baru ini lebih pendek dari yang tercatat sebelumnya, update
            if (gBaru < gScore[newRow][newCol]) {
                gScore[newRow][newCol] = gBaru;
                parent[newRow][newCol] = {sekarang.rowPos, sekarang.colPos};
                antrian.push({newRow, newCol, gBaru, hitungHeuristik(newRow, newCol, target.first, target.second)});
            }
        }
    }

    // kalau target tidak pernah kesentuh, berarti tidak ada jalan
    if (gScore[target.first][target.second] == INT_MAX) return {};

    // susun ulang jalur dari target mundur ke start (pakai data "parent")
    vector<pair<int,int>> jalur;
    pair<int,int> now = target;
    while (now != start) {
        jalur.push_back(now);
        now = parent[now.first][now.second];
    }
    jalur.push_back(start);
    reverse(jalur.begin(), jalur.end());

    return jalur;
}

vector<string> jalurKeArah(vector<pair<int,int>>& jalur) {
    vector<string> arah;
    for (size_t i = 1; i < jalur.size(); i++) {
        int dRow = jalur[i].first  - jalur[i-1].first;
        int dCol = jalur[i].second - jalur[i-1].second;

        if (dRow == -1) arah.push_back("UP");
        else if (dRow == 1) arah.push_back("DOWN");
        else if (dCol == -1) arah.push_back("LEFT");
        else if (dCol == 1) arah.push_back("RIGHT");
    }
    return arah;
}

string gabungArah (vector<string> arah) {
    string hasil = "";
    for (int i = 0; i < arah.size() ; i++) {
        hasil += arah[i];
        if (i != arah.size() - 1) {
            hasil += ", ";
        }
    }
    return hasil;
}

void cetakMaze(vector<vector<char>>& maze, pair<int,int> posisiRobot) {
    int ROWS = maze.size();
    int COLS = maze[0].size();
 
    for (int i = 0; i < ROWS; i++) {
        for (int j = 0; j < COLS; j++) {
            if (i == posisiRobot.first && j == posisiRobot.second) {
                cout << 'R';
            } else {
                cout << maze[i][j];
            }
        }
        cout << "\n";
    }
    cout << "\n";
}

int main() {
    vector<vector<char>> maze = {
        {'#', '#', '#', '#', '#', '#', '#'},
        {'#', 'S', '.', '.', '#', 'G', '#'},
        {'#', '#', '#', '.', '#', '.', '#'},
        {'#', '.', '.', '.', '.', '.', '#'},
        {'#', '.', '#', '#', '#', '.', '#'},
        {'#', '.', '.', 'X', '.', 'F', '#'},
        {'#', '#', '#', '#', '#', '#', '#'}
    };

    int ROWS = maze.size();
    int COLS = maze[0].size();

    pair<int,int> startPos = {-1,-1};
    pair<int,int> flagPos  = {-1,-1};
    pair<int,int> goalPos  = {-1,-1};

    findPos(maze, startPos, flagPos, goalPos, ROWS, COLS);

    
    vector<pair<int,int>> jalurKeFlag = jalankanAstar(maze, startPos, flagPos);
    vector<pair<int,int>> jalurKeGoal = jalankanAstar(maze, flagPos, goalPos);

    if (jalurKeFlag.empty() || jalurKeGoal.empty()) {
        cout << "Tidak ada jalan yang ditemukan!\n";
        return 0;
    }

    vector<string> arahKeFlag = jalurKeArah(jalurKeFlag);
    vector<string> arahKeGoal = jalurKeArah(jalurKeGoal);

    cetakMaze(maze, startPos);


    for (int i = 1; i < jalurKeGoal.size(); i++) {
        cetakMaze(maze, jalurKeGoal[i]);
    }    

    cout<<"MAP LOADED     : "<< ROWS << " x " << COLS << endl;
    cout<<"START POSITION : ("<< startPos.first << "," << startPos.second << ")" << endl; 
    cout<<"PATH TO FLAG   : " << gabungArah(arahKeFlag) << endl << "\n";
    for (int i = 1; i < jalurKeFlag.size(); i++) {
        cetakMaze(maze, jalurKeFlag[i]);
    }    
    cout<<"FLAG CAPTURED  : (" << flagPos.first << "," <<  flagPos.second << ")" << endl;
    cout<<"PATH TO BASE   : " << gabungArah(arahKeGoal) << endl << "\n";
    for (int i = 1; i < jalurKeGoal.size(); i++) {
        cetakMaze(maze, jalurKeGoal[i]);
    }    
    cout<<"BASE REACHED   : (" << goalPos.first << "," <<goalPos.second << ")" << endl;  
    cout<<"MISSION COMPLETE" << endl;
    cout<<"TOTAL MOVES    : " << (arahKeFlag.size() + arahKeGoal.size()) << endl;

    return 0;
}