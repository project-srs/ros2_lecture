#include <sqlite3.h>
#include <sqlite_lecture/relation_data.h>

int exec_callback(void *data, int argc, char **argv, char **azColName)
{
  RelationData *relation_data = (RelationData *)data;
  relation_data->increment();
  for (int i = 0; i < argc; i++)
  {
    relation_data->setAtBack(azColName[i], argv[i]);
  }
  return 0;
};

int main(int argc, char *argv[])
{
  (void)argc;
  (void)argv;

  printf("Start\n");

  sqlite3 *db = NULL;
  if (sqlite3_open("data.db", &db) != SQLITE_OK)
  {
    printf("%s[%s]\n", __func__, sqlite3_errmsg(db));
    return -1;
  }

  // create Table
  if (sqlite3_exec(db, "CREATE TABLE IF NOT EXISTS CountList (name PRIMARY KEY, count);", NULL, NULL, NULL) != SQLITE_OK)
  {
    printf("%s[%s]\n", __func__, sqlite3_errmsg(db));
    return -1;
  }

  // insert record
  if (sqlite3_exec(db, "INSERT OR IGNORE INTO CountList VALUES ('boot', 0);", NULL, NULL, NULL) != SQLITE_OK)
  {
    printf("%s[%s]\n", __func__, sqlite3_errmsg(db));
    return -1;
  }

  // update record
  if (sqlite3_exec(db, "UPDATE CountList SET count = count + 1 WHERE name == 'boot'", NULL, NULL, NULL) != SQLITE_OK)
  {
    printf("%s[%s]\n", __func__, sqlite3_errmsg(db));
    return -1;
  }

  // get & view record
  RelationData relation_data;
  if (sqlite3_exec(db, "SELECT * FROM CountList", exec_callback, (void *)&relation_data, NULL))
  {
    printf("%s[%s]\n", __func__, sqlite3_errmsg(db));
    return -1;
  }
  relation_data.print();

  printf("Done\n");
  return 0;
}