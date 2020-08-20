/**
 * \brief ASRL namespace brief description.
 * 
 * ASRL namespace detailed description.
 */
namespace asrl
{
/**
 * \brief Documentation namespace brief description.
 * 
 * This is the namespace containing the documentation.
 */
namespace documentation
{
/**
 * \brief class brief description.
 * 
 * Detailed class description with a reference to \ref example_tutorial.
 */
class ExampleClass
{
public:
  /**
   * \brief Default constructor.
   * The default constructor throws a table (╯°o°）╯︵ ┻━┻.
   */
  ExampleClass();

protected:
  /**
   * \brief function brief description.
   * 
   * Detailed function description.
   */
  int exampleFunction(
    const int & var1,  ///< input arg 1
    const int & var2,  ///< input arg 2
    int & var3         ///< output arg 1
  );

  /**
   * \brief function brief description.
   * 
   * Detailed function description.
   * \param[in] var1 description of var1
   * \param[in] var2 description of var2
   * \param[out] var3 description of var3
   */
  int exampleFunction2(const int & var1, const int & var2, int & var3);
};

}  // namespace documentation
}  // namespace asrl